"""ROS-free, bounded C2 quintic trajectories for the UR passthrough interface.

Every point carries q/v/a. Only explicit stationary source intervals have zero
velocity.  Ordinary process boundaries remain exact q knots with shared C2
derivatives.  Every quintic control polygon is certified inside a small tube
around its reviewed MoveIt joint-space segment; the resulting spatial curve
MUST still pass a new MoveIt collision and FK-fidelity check.
"""
from __future__ import annotations

import math
import struct

POLICY = "bounded-quintic-v6-source-timed"
# These bounds cover the complete quintic curve, not just knots.  0.2 rad/s is
# the measured peak class of the reviewed UMS5 MoveIt source (0.1954 rad/s),
# rather than the obsolete 0.03 rad/s commissioning cap that made a 12-minute
# path last hours.  Acceleration stays at half the installed MoveIt joint
# limit.  The finite jerk bound retains C2 q/v/a continuity without the previous
# artificial time inflation.  Source timestamps remain lower bounds and every
# complete wire-quantized polynomial must pass derivative certification.
LIMIT_CLASSES = {
    "print": {"velocity": 0.2, "acceleration": 2.5, "jerk": 100.0},
    "approach": {"velocity": 0.2, "acceleration": 2.5, "jerk": 100.0},
}
# One trajectory point per UR3e 500 Hz control update.  This is a supply-floor,
# not a dwell: longer source intervals are retained and may only be stretched.
MINIMUM_SEGMENT_DURATION_S = {"print": 0.002, "approach": 0.002}
# Convex Bernstein certification: every control lies in the L-infinity tube
# around the corresponding reviewed MoveIt q segment.  Hence the entire
# quintic lies in that same convex tube; this is not claimed as a TCP bound.
WAYPOINT_CORRIDOR_RAD = {"print": 0.0012, "approach": 0.0012}
# Subdivide long source segments with points ON the immutable MoveIt q line.
# This makes the C2 temporal blend faithful enough for the independent 0.5 mm
# tool0 FK gate; it does not simplify, shortcut, or move the reviewed path.
MAX_SOURCE_SEGMENT_STEP_RAD = {"print": 0.02, "approach": 0.02}
LIMITS = dict(LIMIT_CLASSES["print"])
FIELDS = ("positions", "velocities", "accelerations")
WIRE_QUANTIZATION = {"joint_scale": 1_000_000, "time_scale": 1_000_000,
                     "time_input": "float32", "rounding": "nearest_ties_away_from_zero",
                     "source": "urcl-2.11.0"}
_WIRE_SCALE = 1_000_000
_WIRE_MAX = 2**31 - 1


class _ContinuousTimingFailure(ValueError):
    """An optional candidate is unavailable; other certified curves may win."""


def _round_away(value):
    return int(math.copysign(math.floor(abs(value) + 0.5), value))


def _wire_joint(value):
    encoded = _round_away(_number(value) * _WIRE_SCALE)
    if not -2**31 <= encoded <= _WIRE_MAX:
        raise ValueError("Joint value is outside the URCL int32 wire range")
    return encoded / _WIRE_SCALE


def _wire_time_us(value):
    value = _number(value)
    if value <= 0.0 or value > _WIRE_MAX / _WIRE_SCALE:
        raise ValueError("Segment duration is outside the positive URCL int32 wire range")
    # UrDriver's float argument narrows the ROS double before the C++ library
    # rounds seconds*1e6. Merely round(value, 6) does not reproduce that API.
    narrowed = struct.unpack("f", struct.pack("f", value))[0]
    encoded = _round_away(narrowed * _WIRE_SCALE)
    if not 0 < encoded <= _WIRE_MAX:
        raise ValueError("Segment duration is outside the positive URCL int32 wire range")
    return encoded


def _wire_time_at_least(value):
    minimum_us = max(1, math.ceil(_number(value) * _WIRE_SCALE))
    candidate_us = minimum_us
    for _ in range(64):
        encoded = _wire_time_us(candidate_us / _WIRE_SCALE)
        if encoded >= minimum_us and _wire_time_us(encoded / _WIRE_SCALE) == encoded:
            return encoded / _WIRE_SCALE
        candidate_us = max(candidate_us + 1, minimum_us + max(0, minimum_us - encoded))
    raise ValueError("Could not represent the segment duration exactly through the URCL float API")


def _number(value):
    if isinstance(value, bool) or not math.isfinite(float(value)):
        raise ValueError("Trajectory contains a non-finite or invalid number")
    return float(value)


def _point_metadata(point):
    """Carry immutable timeline labels through retiming without affecting q/v/a."""
    metadata = {}
    if "path_progress" in point:
        progress = _number(point["path_progress"])
        if not 0.0 <= progress <= 1.0:
            raise ValueError("Trajectory path progress is outside zero through one")
        metadata["path_progress"] = progress
    if "timeline_markers" in point:
        markers = point["timeline_markers"]
        if (
            not isinstance(markers, list)
            or any(not isinstance(marker, str) or not marker or len(marker) > 80 for marker in markers)
            or len(set(markers)) != len(markers)
        ):
            raise ValueError("Trajectory timeline markers are invalid")
        metadata["timeline_markers"] = list(markers)
    return metadata


def controls(a, b, joint):
    """Quintic Bernstein controls for the nominal UR q/v/a polynomial."""
    dt = b["time_s"] - a["time_s"]
    q0, v0, a0 = (a[field][joint] for field in FIELDS)
    q1, v1, a1 = (b[field][joint] for field in FIELDS)
    return [q0, q0 + dt*v0/5, q0 + 2*dt*v0/5 + dt*dt*a0/20,
            q1 - 2*dt*v1/5 + dt*dt*a1/20, q1 - dt*v1/5, q1]


def _control_vectors(a, b):
    per_joint = [controls(a, b, joint) for joint in range(6)]
    return [[per_joint[joint][index] for joint in range(6)] for index in range(6)]


def _inside_reference_tube(value, control_index, start, end, tolerance):
    """Certify a control against the same-u degree-five reference chord.

    A linear q chord elevated to Bernstein degree five has controls
    ``q0 + i/5 * (q1-q0)``.  Bounding every candidate control against that
    corresponding control bounds the complete candidate-minus-reference
    polynomial at the *same* normalized progress, unlike a set-only tube that
    permits arbitrary lag/lead along the chord.
    """
    fraction = control_index/5.0
    return all(
        abs(candidate-(first+fraction*(last-first))) <= tolerance
        for candidate, first, last in zip(value, start, end)
    )


def derivative_bounds(points):
    """Convex-hull bounds cover entire polynomials, not just sampled extrema."""
    peak_v = peak_a = peak_j = 0.0
    for a, b in zip(points, points[1:]):
        dt = b["time_s"] - a["time_s"]
        if not math.isfinite(dt) or dt < 1e-9:
            raise ValueError("Trajectory point times must be strictly increasing")
        velocity_scale = 5.0/dt
        acceleration_scale = 4.0/dt
        jerk_scale = 3.0/dt
        for joint in range(6):
            c0, c1, c2, c3, c4, c5 = controls(a, b, joint)
            v0 = velocity_scale*(c1-c0)
            v1 = velocity_scale*(c2-c1)
            v2 = velocity_scale*(c3-c2)
            v3 = velocity_scale*(c4-c3)
            v4 = velocity_scale*(c5-c4)
            a0 = acceleration_scale*(v1-v0)
            a1 = acceleration_scale*(v2-v1)
            a2 = acceleration_scale*(v3-v2)
            a3 = acceleration_scale*(v4-v3)
            j0 = jerk_scale*(a1-a0)
            j1 = jerk_scale*(a2-a1)
            j2 = jerk_scale*(a3-a2)
            peak_v = max(peak_v, abs(v0), abs(v1), abs(v2), abs(v3), abs(v4))
            peak_a = max(peak_a, abs(a0), abs(a1), abs(a2), abs(a3))
            peak_j = max(peak_j, abs(j0), abs(j1), abs(j2))
    return {"velocity": peak_v, "acceleration": peak_a, "jerk": peak_j}


def validate_profile(profile, *, expected_limit_class=None):
    limit_class = profile.get("limit_class")
    limits = LIMIT_CLASSES.get(limit_class)
    if profile.get("policy") != POLICY or limits is None or profile.get("limits") != limits:
        raise ValueError("Obsolete or altered smooth trajectory policy; prepare again")
    if expected_limit_class is not None and limit_class != expected_limit_class:
        raise ValueError(
            f"Smooth trajectory limit class must be {expected_limit_class}; prepare again"
        )
    if profile.get("wire_quantization") != WIRE_QUANTIZATION:
        raise ValueError("Smooth trajectory has no matching URCL wire-quantization certificate; prepare again")
    corridor = WAYPOINT_CORRIDOR_RAD.get(limit_class)
    minimum_segment = MINIMUM_SEGMENT_DURATION_S.get(limit_class)
    maximum_source_step = MAX_SOURCE_SEGMENT_STEP_RAD.get(limit_class)
    if profile.get("waypoint_corridor_rad") != corridor:
        raise ValueError("Smooth trajectory has no matching source-path corridor certificate")
    if profile.get("minimum_segment_duration_s") != minimum_segment:
        raise ValueError("Smooth trajectory has no matching controller update-rate certificate")
    if profile.get("maximum_source_segment_step_rad") != maximum_source_step:
        raise ValueError("Smooth trajectory has no matching source-segment sampling certificate")
    points = profile.get("points", [])
    duration = _number(profile["duration_s"])
    if duration <= 0.0:
        raise ValueError("Smooth trajectory duration must be positive and finite")
    if len(points) < 2 or len(points) > 1_000_000:
        raise ValueError("Smooth trajectory point count is invalid")
    for point in points:
        _number(point["time_s"])
        _point_metadata(point)
        for field in FIELDS:
            if len(point[field]) != 6:
                raise ValueError("A smooth point requires six positions, velocities and accelerations")
            for value in point[field]:
                _number(value)
                if value != _wire_joint(value):
                    raise ValueError("Smooth q/v/a must already match the quantized URCL wire values")
    if points[0]["time_s"] != 0 or abs(points[-1]["time_s"] - duration) > 1e-9:
        raise ValueError("Smooth trajectory duration is inconsistent")
    if any(abs(value) > 1e-12 for p in (points[0], points[-1])
           for f in ("velocities", "accelerations") for value in p[f]):
        raise ValueError("Each independently dispatched trajectory must start and end at rest")
    peaks = derivative_bounds(points)
    for a, b in zip(points, points[1:]):
        if b["time_s"]-a["time_s"] < minimum_segment-1.0e-9:
            raise ValueError("Smooth trajectory segment is faster than the UR3e update rate")
        if max(abs(x-y) for x, y in zip(a["positions"], b["positions"])) \
                > maximum_source_step+2.0/_WIRE_SCALE:
            raise ValueError("Smooth trajectory source segment exceeds its sampling certificate")
        for control_index, control in enumerate(_control_vectors(a, b)):
            if not _inside_reference_tube(
                control, control_index, a["positions"], b["positions"],
                corridor+1.0e-10
            ):
                raise ValueError("UR curve leaves its certified same-progress source corridor")
    # The hardware interface differences absolute ROS timestamps, then passes
    # each segment through float32 and integer microseconds. Check that decoding
    # those bytes reconstructs the very curve reviewed above, not a nearby one.
    elapsed_us = 0
    for a, b in zip(points, points[1:]):
        elapsed_us += _wire_time_us(b["time_s"] - a["time_s"])
        if abs(b["time_s"] - elapsed_us / _WIRE_SCALE) > 1e-9:
            raise ValueError("Smooth times change after URCL float32/microsecond quantization")
    if any(not math.isfinite(peaks[k]) or peaks[k] > limits[k]*(1+1e-7) for k in limits):
        raise ValueError("UR interpolated velocity, acceleration or jerk exceeds its limit")
    return peaks


def stretch_profile(profile, duration_s):
    """Retiming with wire rounding may adjust tangents; collision-check the result.

    Required stop boundaries and quantized q waypoints are preserved. This is
    not an exact uniform-dilation certificate for a previously reviewed curve.
    """
    validate_profile(profile)
    duration_s = _number(duration_s)
    if duration_s < profile["duration_s"] - 1e-9:
        raise ValueError("A bounded trajectory may only be slowed, not sped up")
    scale = max(1.0, duration_s / profile["duration_s"])
    return compile_profile([{"time_s": point["time_s"] * scale,
                             "positions": list(point["positions"]),
                             **_point_metadata(point)}
                            for point in profile["points"]], duration_s,
                           limit_class=profile["limit_class"])


def _inverse2(matrix):
    a, b = matrix[0]
    c, d = matrix[1]
    determinant = a*d - b*c
    if not math.isfinite(determinant) or determinant <= 0:
        raise _ContinuousTimingFailure("Continuous timing system is not positive definite")
    return [[d/determinant, -b/determinant], [-c/determinant, a/determinant]]


def _multiply2(left, right):
    return [[sum(left[i][k]*right[k][j] for k in range(2))
             for j in range(len(right[0]))] for i in range(2)]


def _subtract2(left, right):
    return [[x-y for x, y in zip(a, b)] for a, b in zip(left, right)]


def _minimum_jerk_states(q, intervals):
    """Shared interior v/a minimizing integral jerk², with endpoints at rest.

    Positions and source times are fixed. The 2x2 block-tridiagonal Hessian
    comes from analytically integrating the quintic jerk on each interval.
    Unlike the old construction, dense IK points are not acceleration stops.
    The result is only a candidate: shape projection and full wire-curve
    derivative certification below remain mandatory.
    """
    count = len(q)-2
    zeros = [[0.0]*6 for _ in q]
    if count == 0:
        return zeros, [[0.0]*6 for _ in q]
    diagonal = [[[0.0]*2 for _ in range(2)] for _ in range(count)]
    rhs = [[[0.0]*6 for _ in range(2)] for _ in range(count)]
    upper = [[[0.0]*2 for _ in range(2)] for _ in range(count-1)]
    for i, h in enumerate(intervals):
        delta = [y-x for x, y in zip(q[i], q[i+1])]
        left = [[192/h**3, 36/h**2], [36/h**2, 9/h]]
        right = [[192/h**3, -36/h**2], [-36/h**2, 9/h]]
        for knot, block, linear in ((i, left, (360/h**4, 60/h**3)),
                                     (i+1, right, (360/h**4, -60/h**3))):
            if 0 < knot < len(q)-1:
                index = knot-1
                for row in range(2):
                    for col in range(2):
                        diagonal[index][row][col] += block[row][col]
                    for joint in range(6):
                        rhs[index][row][joint] += linear[row]*delta[joint]
        if 0 < i < count:
            upper[i-1] = [[168/h**3, -24/h**2], [24/h**2, -3/h]]
    # Block Thomas elimination; six joint right-hand sides share one solve.
    forward, values = [], []
    for i in range(count):
        if i:
            lower = [list(row) for row in zip(*upper[i-1])]
            diagonal[i] = _subtract2(diagonal[i], _multiply2(lower, forward[-1]))
            rhs[i] = _subtract2(rhs[i], _multiply2(lower, values[-1]))
        inverse = _inverse2(diagonal[i])
        values.append(_multiply2(inverse, rhs[i]))
        if i < count-1:
            forward.append(_multiply2(inverse, upper[i]))
    states = [None]*count
    states[-1] = values[-1]
    for i in range(count-2, -1, -1):
        states[i] = _subtract2(values[i], _multiply2(forward[i], states[i+1]))
    return ([[0.0]*6] + [state[0] for state in states] + [[0.0]*6],
            [[0.0]*6] + [state[1] for state in states] + [[0.0]*6])


def _truncate_joint(value):
    return math.copysign(math.floor(abs(value)*_WIRE_SCALE)/_WIRE_SCALE, value)


def _phase_state(
    dp, dn, hp, hn, target, desired_velocity=0.0, desired_acceleration=0.0,
    *, preserve_direction=False,
):
    """Return one feasible (v,a) for the four same-u controls, or None."""
    if preserve_direction and dp*dn > 0.0 and dp*desired_velocity <= 0.0:
        # A global minimum-jerk candidate can point backwards at an otherwise
        # monotone source knot.  Clamping that candidate to zero invents a
        # momentary process-boundary stop.  Prefer the source secant instead;
        # the exact feasible corridor below still clips/certifies the state.
        desired_velocity = (dp+dn)/(hp+hn)
    direction_ok = (
        not preserve_direction
        or (dp > 0.0 and dn > 0.0 and desired_velocity >= 0.0)
        or (dp < 0.0 and dn < 0.0 and desired_velocity <= 0.0)
        or dp == 0.0
        or dn == 0.0
        or dp*dn < 0.0
    )
    if (
        direction_ok
        and
        abs(hp*desired_velocity-dp) <= 5*target
        and abs(hn*desired_velocity-dn) <= 5*target
        and abs(-8*hp*desired_velocity+hp*hp*desired_acceleration+8*dp)
            <= 20*target
        and abs(8*hn*desired_velocity+hn*hn*desired_acceleration-8*dn)
            <= 20*target
    ):
        return desired_velocity, desired_acceleration
    lower = max((dp-5*target)/hp, (dn-5*target)/hn)
    upper = min((dp+5*target)/hp, (dn+5*target)/hn)
    if preserve_direction and dp > 0.0 and dn > 0.0:
        lower = max(lower, 0.0)
    elif preserve_direction and dp < 0.0 and dn < 0.0:
        upper = min(upper, 0.0)
    if lower > upper:
        if lower-upper > 1.0e-10*max(1.0, abs(lower), abs(upper)):
            return None
        lower = upper = (lower+upper)/2.0
    lowers = (
        (8/hp, -8*dp/(hp*hp)-20*target/(hp*hp)),
        (-8/hn, 8*dn/(hn*hn)-20*target/(hn*hn)),
    )
    uppers = (
        (8/hp, -8*dp/(hp*hp)+20*target/(hp*hp)),
        (-8/hn, 8*dn/(hn*hn)+20*target/(hn*hn)),
    )
    for lm, lb in lowers:
        for um, ub in uppers:
            coefficient = lm-um
            rhs = ub-lb
            if abs(coefficient) <= 1.0e-18:
                if rhs < 0.0:
                    return None
            elif coefficient > 0.0:
                upper = min(upper, rhs/coefficient)
            else:
                lower = max(lower, rhs/coefficient)
    if lower > upper:
        if lower-upper > 1.0e-10*max(1.0, abs(lower), abs(upper)):
            return None
        lower = upper = (lower+upper)/2.0
    velocity = min(upper, max(lower, desired_velocity))
    acceleration_lower = max(m*velocity+b for m, b in lowers)
    acceleration_upper = min(m*velocity+b for m, b in uppers)
    if acceleration_lower > acceleration_upper:
        if acceleration_lower-acceleration_upper > 1.0e-10*max(
            1.0, abs(acceleration_lower), abs(acceleration_upper)
        ):
            return None
        acceleration_lower = acceleration_upper = (
            acceleration_lower+acceleration_upper
        )/2.0
    acceleration = min(acceleration_upper, max(acceleration_lower, desired_acceleration))
    return velocity, acceleration


def _corridor_state_target(tolerance, hp, hn):
    """Leave enough room for the final 1e-6 wire rounding of shared v/a.

    Position knots and times are already on their wire lattices.  Only v/a are
    rounded after the continuous solve.  The P2/P3 controls have the largest
    amplification, so reserve their exact worst-case half-quantum error rather
    than relying on a fixed margin that becomes unsafe on long intervals.
    """
    longest = max(hp, hn)
    rounding = longest*2.0e-7 + longest*longest*2.5e-8
    target = tolerance-rounding-1.0e-10
    if target <= 0.0:
        raise _ContinuousTimingFailure(
            "A source interval is too long for the wire-quantized corridor"
        )
    return target


def _repair_phase_timing(q, intervals, tolerance):
    """Stretch only intervals needed for adjacent same-u C2 feasibility."""
    def knot_feasible(knot, hp, hn):
        try:
            target = _corridor_state_target(tolerance, hp, hn)
        except _ContinuousTimingFailure:
            return False
        return all(
            _phase_state(
                q[knot][joint]-q[knot-1][joint],
                q[knot+1][joint]-q[knot][joint], hp, hn, target,
                preserve_direction=True,
            ) is not None
            for joint in range(6)
        )

    for _ in range(32):
        requested = [1.0]*len(intervals)
        repaired = False
        for knot in range(1, len(q)-1):
            hp, hn = intervals[knot-1], intervals[knot]
            if knot_feasible(knot, hp, hn):
                continue
            options = []
            for side in (0, 1):
                # Feasibility is a function of the adjacent timing *ratio*,
                # not a monotone function of duration.  Doubling 1,2,4,...
                # can jump completely over a narrow valid ratio window (the
                # UMS5 regression has a real 1.14--1.36 example).  Scan one
                # thirty-second of an octave at a time, then bisect only the
                # observed fail->pass bracket.  Stop once the stretched
                # interval would exceed the exact URCL duration range.
                previous = 1.0
                maximum = (_WIRE_MAX/_WIRE_SCALE)/(hp if side == 0 else hn)
                octave_steps = max(1, int(math.ceil(32.0*math.log2(maximum))))
                for step in range(1, octave_steps+1):
                    upper = min(maximum, 2.0**(step/32.0))
                    first = hp*upper if side == 0 else hp
                    second = hn*upper if side == 1 else hn
                    if knot_feasible(knot, first, second):
                        lower = previous
                        for _binary in range(24):
                            middle = (lower+upper)/2.0
                            first = hp*middle if side == 0 else hp
                            second = hn*middle if side == 1 else hn
                            if knot_feasible(knot, first, second):
                                upper = middle
                            else:
                                lower = middle
                        options.append((upper, side))
                        break
                    if upper >= maximum:
                        break
                    previous = upper
            if not options:
                incoming_step = max(
                    abs(q[knot][joint]-q[knot-1][joint]) for joint in range(6)
                )
                outgoing_step = max(
                    abs(q[knot+1][joint]-q[knot][joint]) for joint in range(6)
                )
                raise _ContinuousTimingFailure(
                    f"Source corner {knot} has no same-progress timing "
                    f"(dt={hp:.9g}/{hn:.9g}s, "
                    f"step={incoming_step:.9g}/{outgoing_step:.9g}rad)"
                )
            factor, side = min(options)
            intervals_to_stretch = (knot-1 if side == 0 else knot,)
            for interval in intervals_to_stretch:
                requested[interval] = max(requested[interval], factor*1.0005)
            repaired = True
        if not repaired:
            return
        for index, factor in enumerate(requested):
            if factor > 1.0:
                intervals[index] = _wire_time_at_least(intervals[index]*factor)
    raise _ContinuousTimingFailure("Same-progress timing repair did not converge")


def _project_corridor(points, tolerance, *, quantize):
    """Project each shared knot state into its exact same-u convex corridor.

    For one knot/joint, the two adjacent quintics constrain only that knot's
    shared ``(v, a)``.  Each Bernstein bound is a linear strip, so feasibility
    reduces to a one-dimensional velocity interval followed by an acceleration
    interval.  This is O(points*joints), deterministic, and preserves C2.
    """
    for knot in range(1, len(points)-1):
        previous, point, following = points[knot-1:knot+2]
        hp = point["time_s"]-previous["time_s"]
        hn = following["time_s"]-point["time_s"]
        if hp <= 0.0 or hn <= 0.0:
            raise _ContinuousTimingFailure("Non-positive interval in corridor projection")
        target = _corridor_state_target(tolerance, hp, hn)
        for joint in range(6):
            dp = point["positions"][joint]-previous["positions"][joint]
            dn = following["positions"][joint]-point["positions"][joint]
            state = _phase_state(
                dp, dn, hp, hn, target,
                point["velocities"][joint], point["accelerations"][joint],
                preserve_direction=True,
            )
            if state is None:
                raise _ContinuousTimingFailure(
                    f"No same-progress C2 state exists at source knot {knot}, joint {joint}"
                )
            velocity, acceleration = state
            point["velocities"][joint] = _wire_joint(velocity) if quantize else velocity
            point["accelerations"][joint] = (
                _wire_joint(acceleration) if quantize else acceleration
            )

    if not quantize:
        return
    for a, b in zip(points, points[1:]):
        for control_index, control in enumerate(_control_vectors(a, b)):
            if not _inside_reference_tube(
                control, control_index, a["positions"], b["positions"], tolerance-1.0e-10
            ):
                reference = [
                    first+control_index/5.0*(last-first)
                    for first, last in zip(a["positions"], b["positions"])
                ]
                error = max(abs(x-y) for x, y in zip(control, reference))
                raise _ContinuousTimingFailure(
                    "Quantized shared state cannot satisfy its same-progress corridor "
                    f"({error:.9g} rad)"
                )


def _allocate_interval_group(weights, lower_bound, minimum_segment):
    """Allocate one source chord across line-preserving support intervals."""
    durations = [max(minimum_segment, _number(weight)) for weight in weights]
    lower_bound = max(_number(lower_bound), minimum_segment*len(durations))
    total = sum(durations)
    if total < lower_bound:
        scale = lower_bound/total
        durations = [duration*scale for duration in durations]
    return [_wire_time_at_least(duration) for duration in durations]


def _continuous_profile(
    path,
    minimum_duration_s,
    limit_class,
    *,
    local_retiming,
    progress_callback=None,
):
    def report(fraction):
        if progress_callback is not None:
            progress_callback(max(0.0, min(1.0, float(fraction))))

    report(0.0)
    limits = LIMIT_CLASSES[limit_class]
    minimum_segment = MINIMUM_SEGMENT_DURATION_S[limit_class]
    corridor = WAYPOINT_CORRIDOR_RAD[limit_class]
    path = [dict(point, positions=list(point["positions"])) for point in path]
    source_q = [[_wire_joint(value) for value in point["positions"]] for point in path]
    source_intervals = [
        max(b["time_s"]-a["time_s"], minimum_segment)
        for a, b in zip(path, path[1:])
    ]
    minimum_scale = max(1.0, minimum_duration_s/sum(source_intervals))
    source_intervals = [duration*minimum_scale for duration in source_intervals]

    # The same-u corridor is a local convex constraint on each knot state.
    # Most dense MoveIt knots already have a feasible shared v/a.  Add support
    # points only around a true infeasible corner (and stopped island ends),
    # always on the immutable source chord.  The former all-chord expansion
    # tripled UMS5's point count and manufactured thousands of 2 ms jerk traps.
    phase_knots = {0, len(path)-1}
    for knot in range(1, len(path)-1):
        hp, hn = source_intervals[knot-1:knot+1]
        target = _corridor_state_target(corridor, hp, hn)
        if any(
            _phase_state(
                source_q[knot][joint]-source_q[knot-1][joint],
                source_q[knot+1][joint]-source_q[knot][joint],
                hp, hn, target,
            ) is None
            for joint in range(6)
        ):
            phase_knots.add(knot)
    report(0.15)
    phase_step = max(1.0e-6, 2.0*(corridor-2.0e-6))

    def interpolated(first, last, fraction):
        result = {
            "time_s": first["time_s"]+fraction*(last["time_s"]-first["time_s"]),
            "positions": [
                a+fraction*(b-a) for a, b in zip(first["positions"], last["positions"])
            ],
        }
        if "path_progress" in first and "path_progress" in last:
            result["path_progress"] = first["path_progress"]+fraction*(
                last["path_progress"]-first["path_progress"]
            )
        return result

    refined = [path[0]]
    interval_groups = []
    interval_fractions = []
    for source_interval, (first, last) in enumerate(zip(path, path[1:])):
        step = max(abs(b-a) for a, b in zip(first["positions"], last["positions"]))
        fractions = []
        if step > phase_step:
            fraction = phase_step/step
            if source_interval in phase_knots:
                fractions.append(fraction)
            if source_interval+1 in phase_knots:
                fractions.append(1.0-fraction)
        fractions = sorted(set(fraction for fraction in fractions if 0.0 < fraction < 1.0))
        if len(fractions) == 2 and fractions[0] >= fractions[1]:
            fractions = [0.5]
        previous_fraction = 0.0
        for fraction in fractions+[1.0]:
            addition = last if fraction == 1.0 else interpolated(first, last, fraction)
            refined.append(addition)
            interval_groups.append(source_interval)
            interval_fractions.append(fraction-previous_fraction)
            previous_fraction = fraction
    path = refined
    report(0.30)
    q = [[_wire_joint(v) for v in p["positions"]] for p in path]
    group_members = [[] for _ in source_intervals]
    for index, group in enumerate(interval_groups):
        group_members[group].append(index)
    delta_t = [0.0]*len(interval_groups)
    for group, members in enumerate(group_members):
        allocated = _allocate_interval_group(
            [source_intervals[group]*interval_fractions[index] for index in members],
            source_intervals[group], minimum_segment,
        )
        for index, duration in zip(members, allocated):
            delta_t[index] = duration

    for iteration in range(64):
        _repair_phase_timing(q, delta_t, corridor)
        velocities, accelerations = _minimum_jerk_states(q, delta_t)
        elapsed_us, points = 0, []
        for i, position in enumerate(q):
            if i:
                elapsed_us += _wire_time_us(delta_t[i-1])
            points.append({"time_s": elapsed_us/_WIRE_SCALE, "positions": position,
                           "velocities": [_truncate_joint(v) for v in velocities[i]],
                           # Projection/retiming may begin from a mathematically
                           # large state when a source interval is only 2 ms.
                           # Truncate first; the successful certified iteration
                           # is checked against the int32 wire range below.
                           "accelerations": [_truncate_joint(a) for a in accelerations[i]],
                           **_point_metadata(path[i])})
        _project_corridor(points, corridor, quantize=False)
        factors = []
        certified_bounds = dict.fromkeys(limits, 0.0)
        for a, b in zip(points, points[1:]):
            peaks = derivative_bounds([a, b])
            for name in limits:
                certified_bounds[name] = max(certified_bounds[name], peaks[name])
            factors.append(max(peaks["velocity"]/limits["velocity"],
                               math.sqrt(peaks["acceleration"]/limits["acceleration"]),
                               (peaks["jerk"]/limits["jerk"])**(1/3)))
        if max(factors) <= 1.00000001:
            _project_corridor(points, corridor, quantize=True)
            factors = []
            certified_bounds = dict.fromkeys(limits, 0.0)
            for a, b in zip(points, points[1:]):
                peaks = derivative_bounds([a, b])
                for name in limits:
                    certified_bounds[name] = max(certified_bounds[name], peaks[name])
                factors.append(max(peaks["velocity"]/limits["velocity"],
                                   math.sqrt(peaks["acceleration"]/limits["acceleration"]),
                                   (peaks["jerk"]/limits["jerk"])**(1/3)))
        if max(factors) <= 1.00000001:
            profile = {
                "policy": POLICY,
                "limit_class": limit_class,
                "limits": dict(limits),
                "points": points,
                "duration_s": points[-1]["time_s"],
                "wire_quantization": dict(WIRE_QUANTIZATION),
                "waypoint_corridor_rad": corridor,
                "minimum_segment_duration_s": minimum_segment,
                "maximum_source_segment_step_rad": MAX_SOURCE_SEGMENT_STEP_RAD[limit_class],
                "timing_method": "continuous-minimum-jerk-" + (
                    "local" if local_retiming else "uniform"
                ),
            }
            # The outer compiler validates the complete assembled trajectory,
            # including holds, once.  These bounds were already computed on
            # this exact quantized island; repeating its full spatial/wire
            # certificate here wastes 164 validations on the UMS5 fixture.
            profile["bounds"] = certified_bounds
            report(1.0)
            return profile
        if not local_retiming:
            factor = max(1.0, max(factors))
            delta_t = [_wire_time_at_least(h*factor*1.0005) for h in delta_t]
        elif len(delta_t) < 128 and iteration >= 8:
            # Tiny adversarial paths have no useful duration budget to
            # redistribute.  A uniform dilation preserves every normalized
            # control and therefore converges in one further certification.
            factor = max(1.0, max(factors))*1.002
            delta_t = [_wire_time_at_least(h*factor) for h in delta_t]
        else:
            # A support point subdivides one immutable source chord; it does
            # not create a new source-time lower bound.  Reallocate time among
            # those children according to their measured v/a/j demand, while
            # keeping every original chord's total duration as a lower bound.
            requested = [
                duration*max(1.0, factor*1.015)
                for duration, factor in zip(delta_t, factors)
            ]
            for group, members in enumerate(group_members):
                allocated = _allocate_interval_group(
                    [requested[index] for index in members],
                    source_intervals[group], minimum_segment,
                )
                for index, duration in zip(members, allocated):
                    delta_t[index] = duration
        report(0.35 + 0.60 * (iteration + 1) / 64.0)
    raise _ContinuousTimingFailure("Continuous bounded trajectory retiming did not converge")


def compile_profile(
    path,
    minimum_duration_s=0.0,
    *,
    limit_class="print",
    progress_callback=None,
):
    """Compile source-timed C2 motion islands separated only by real holds.

    Every ordinary process boundary remains an exact q knot inside one island;
    it therefore shares v/a and cannot manufacture a block pause.  An adjacent
    pair of identical source positions is an explicit stationary interval and
    is the only place an island is stopped.  If the continuous curve cannot be
    certified, preparation fails closed instead of silently selecting the old
    multi-hour zero-acceleration fallback.
    """
    def report(fraction, completed=None, total=None):
        if progress_callback is not None:
            progress_callback(
                max(0.0, min(1.0, float(fraction))),
                completed,
                total,
            )

    report(0.0, 0, max(1, len(path) - 1))
    if len(path) < 2:
        raise ValueError("A trajectory needs at least two source points")
    if limit_class not in LIMIT_CLASSES:
        raise ValueError("Unknown smooth trajectory limit class")
    source_times = [_number(point["time_s"]) for point in path]
    if any(b <= a for a, b in zip(source_times, source_times[1:])):
        raise ValueError("Invalid source trajectory timing")
    minimum_duration_s = _number(minimum_duration_s)
    if minimum_duration_s < 0.0:
        raise ValueError("Minimum trajectory duration must not be negative")
    source_duration = source_times[-1]-source_times[0]
    source_scale = max(1.0, minimum_duration_s/source_duration)
    normalized_path = [
        {
            "time_s": (source_times[index]-source_times[0])*source_scale,
            "positions": list(point["positions"]),
            **_point_metadata(point),
        }
        for index, point in enumerate(path)
    ]
    wire_q = [[_wire_joint(v) for v in point["positions"]] for point in path]
    continuous_path = [normalized_path[0]]
    for i in range(1, len(path)-1):
        neighbours = (i-1, i+1)
        explicit_hold = any(path[i]["positions"] == path[neighbour]["positions"]
                            for neighbour in neighbours)
        unmarked = not _point_metadata(path[i]).get("timeline_markers")
        redundant_wire_knot = any(
            wire_q[i] == wire_q[neighbour]
            and path[i]["positions"] != path[neighbour]["positions"]
            for neighbour in neighbours
        )
        # A block marker can be only microseconds from the MoveIt sample it
        # interpolates.  Both cannot be supplied in one 500 Hz cycle.  Keep the
        # exact process marker and remove only the unlabelled neighbour when
        # their q separation itself fits inside the certified source corridor.
        # This avoids inventing a 2 ms zig-zag or an ill-conditioned spline;
        # every labelled q boundary and every larger MoveIt feature remains.
        redundant_near_marker = any(
            bool(_point_metadata(path[neighbour]).get("timeline_markers"))
            and abs(source_times[i]-source_times[neighbour])
                < MINIMUM_SEGMENT_DURATION_S[limit_class]
            and max(abs(a-b) for a, b in zip(wire_q[i], wire_q[neighbour]))
                <= WAYPOINT_CORRIDOR_RAD[limit_class]
            for neighbour in neighbours
        )
        redundant = unmarked and not explicit_hold and (
            redundant_wire_knot or redundant_near_marker
        )
        if not redundant:
            continuous_path.append(normalized_path[i])
    continuous_path.append(normalized_path[-1])
    report(0.12, len(continuous_path) - 1, max(1, len(path) - 1))

    certified_path = [continuous_path[0]]
    maximum_step = MAX_SOURCE_SEGMENT_STEP_RAD[limit_class]
    # The unified print path carries immutable process markers. Preserve a
    # denser joint reference for its independent tool0 FK gate without forcing
    # approaches, already-compiled profiles or ROS-free utility paths through
    # unnecessary support knots. The serialized 0.02 certificate remains
    # conservative.
    support_step = min(
        maximum_step,
        0.02 if any(point.get("timeline_markers") for point in continuous_path)
        else maximum_step,
    )
    for first, last in zip(continuous_path, continuous_path[1:]):
        delta = [b-a for a, b in zip(first["positions"], last["positions"])]
        divisions = max(1, int(math.ceil(max(abs(value) for value in delta)/support_step)))
        # An explicit hold must remain one stationary interval with its exact
        # source/stepper duration.  Moving intervals are split only on their
        # original straight q segment and at proportional source timestamps.
        if first["positions"] == last["positions"]:
            divisions = 1
        for part in range(1, divisions+1):
            if part == divisions:
                certified_path.append(last)
                continue
            fraction = part/divisions
            point = {
                "time_s": first["time_s"] + fraction*(last["time_s"]-first["time_s"]),
                "positions": [a+fraction*d for a, d in zip(first["positions"], delta)],
            }
            if "path_progress" in first and "path_progress" in last:
                point["path_progress"] = first["path_progress"] + fraction*(
                    last["path_progress"]-first["path_progress"]
                )
            certified_path.append(point)
    continuous_path = certified_path
    total_work = max(1, len(continuous_path) - 1)
    report(0.22, 0, total_work)

    holds = [
        index
        for index, (a, b) in enumerate(zip(continuous_path, continuous_path[1:]))
        if a["positions"] == b["positions"]
    ]
    points = []
    elapsed_us = 0
    island_count = 0

    def append_point(point, absolute_time_s):
        candidate = {
            "time_s": absolute_time_s,
            "positions": list(point["positions"]),
            "velocities": list(point["velocities"]),
            "accelerations": list(point["accelerations"]),
            **_point_metadata(point),
        }
        if points and abs(points[-1]["time_s"]-absolute_time_s) <= 1.0e-12:
            if points[-1]["positions"] != candidate["positions"]:
                raise _ContinuousTimingFailure("Motion islands do not share one q boundary")
            for marker in candidate.get("timeline_markers", []):
                markers = points[-1].setdefault("timeline_markers", [])
                if marker not in markers:
                    markers.append(marker)
            if "path_progress" in candidate:
                points[-1]["path_progress"] = candidate["path_progress"]
            return
        points.append(candidate)

    def append_island(start, end):
        nonlocal elapsed_us, island_count
        if end <= start:
            return
        def island_progress(fraction):
            completed = min(total_work, start + (end - start) * fraction)
            report(
                0.25 + 0.68 * completed / total_work,
                int(completed),
                total_work,
            )

        island = _continuous_profile(
            continuous_path[start:end+1],
            0.0,
            limit_class,
            local_retiming=True,
            progress_callback=island_progress,
        )
        island_count += 1
        base_us = elapsed_us
        for point in island["points"]:
            append_point(point, (base_us+_round_away(point["time_s"]*_WIRE_SCALE))/_WIRE_SCALE)
        elapsed_us += _round_away(island["duration_s"]*_WIRE_SCALE)

    start = 0
    minimum_segment = MINIMUM_SEGMENT_DURATION_S[limit_class]
    for hold in holds:
        append_island(start, hold)
        if not points:
            append_point(
                {
                    **continuous_path[hold],
                    "positions": [_wire_joint(value) for value in continuous_path[hold]["positions"]],
                    "velocities": [0.0]*6,
                    "accelerations": [0.0]*6,
                },
                elapsed_us/_WIRE_SCALE,
            )
        hold_duration = max(
            continuous_path[hold+1]["time_s"]-continuous_path[hold]["time_s"],
            minimum_segment,
        )
        elapsed_us += _wire_time_us(_wire_time_at_least(hold_duration))
        append_point(
            {
                **continuous_path[hold+1],
                "positions": [_wire_joint(value) for value in continuous_path[hold+1]["positions"]],
                "velocities": [0.0]*6,
                "accelerations": [0.0]*6,
            },
            elapsed_us/_WIRE_SCALE,
        )
        start = hold+1
        report(
            0.25 + 0.68 * min(total_work, start) / total_work,
            min(total_work, start),
            total_work,
        )
    append_island(start, len(continuous_path)-1)

    if len(points) < 2:
        raise ValueError("A trajectory needs at least one positive-duration segment")
    profile = {
        "policy": POLICY,
        "limit_class": limit_class,
        "limits": dict(LIMIT_CLASSES[limit_class]),
        "points": points,
        "duration_s": points[-1]["time_s"],
        "wire_quantization": dict(WIRE_QUANTIZATION),
        "waypoint_corridor_rad": WAYPOINT_CORRIDOR_RAD[limit_class],
        "minimum_segment_duration_s": minimum_segment,
        "timing_method": "piecewise-continuous-minimum-jerk-local",
        "motion_island_count": island_count,
        "stationary_hold_count": len(holds),
        "maximum_source_segment_step_rad": maximum_step,
    }
    report(0.95, total_work, total_work)
    profile["bounds"] = validate_profile(profile)
    profile["source_point_count"] = len(path)
    report(1.0, total_work, total_work)
    return profile


def _local_profile(path, minimum_duration_s=0.0, *, limit_class="print"):
    if len(path) < 2:
        raise ValueError("A trajectory needs at least two source points")
    if limit_class not in LIMIT_CLASSES:
        raise ValueError("Unknown smooth trajectory limit class")
    limits = LIMIT_CLASSES[limit_class]
    times = [_number(p["time_s"]) for p in path]
    q = [[_wire_joint(v) for v in p["positions"]] for p in path]
    if any(len(p) != 6 for p in q) or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Invalid source trajectory dimensions or timing")
    # A clipped boundary can be nanoseconds from a source waypoint. Retiming
    # the whole path from that interval's peak makes ordinary moves last hours.
    # Allocate time locally first, retaining source intervals as lower bounds.
    minimum_segment = MINIMUM_SEGMENT_DURATION_S[limit_class]
    delta_t = [max(b-a, minimum_segment) for a,b in zip(times,times[1:])]
    minimum_duration_s = _number(minimum_duration_s)
    if minimum_duration_s < 0.0:
        raise ValueError("Minimum trajectory duration must not be negative")
    minimum_scale = max(1.0, minimum_duration_s / sum(delta_t))
    delta_t = [dt * minimum_scale for dt in delta_t]
    # Updating one interval changes the monotone tangent shared with each
    # neighbour.  On a long curved joint-space path, the resulting constraint
    # adjustment can therefore advance by only one interval per pass.  Budget
    # one pass per interval (while retaining the historical minimum) instead
    # of falsely rejecting valid MoveIt approaches after a fixed 64 passes.
    retiming_passes = max(64, len(delta_t))
    for iteration in range(retiming_passes):
        delta_t = [_wire_time_at_least(dt) for dt in delta_t]
        times = [0.0]
        elapsed_us = 0
        for dt in delta_t:
            elapsed_us += _wire_time_us(dt)
            times.append(elapsed_us / _WIRE_SCALE)
        slopes = [[(b[j]-a[j])/dt for j in range(6)] for a,b,dt in zip(q,q[1:],delta_t)]
        velocities = [[0.0]*6 for _ in q]
        for i in range(1, len(q)-1):
            for j in range(6):
                left, right = slopes[i-1][j], slopes[i][j]
                if left*right > 0:
                    # Limiting to both secants orders all Bernstein controls,
                    # so each joint stays inside its waypoint interval.
                    tangent = min(abs(left), abs(right))
                    velocities[i][j] = math.copysign(math.floor(tangent * _WIRE_SCALE) / _WIRE_SCALE, left)
        points = [{"time_s": t, "positions": p, "velocities": v,
                   "accelerations": [0.0]*6, **_point_metadata(source)}
                  for t,p,v,source in zip(times,q,velocities,path)]
        factors = []
        for a,b in zip(points, points[1:]):
            local = derivative_bounds([a,b])
            factors.append(max(1.0, local["velocity"]/limits["velocity"],
                               math.sqrt(local["acceleration"]/limits["acceleration"]),
                               (local["jerk"]/limits["jerk"])**(1/3)))
        if max(factors) <= 1.00000001:
            break
        delta_t = [dt * factor * 1.000001 if factor > 1.0 else dt
                   for dt,factor in zip(delta_t,factors)]
    else:
        raise ValueError("Local bounded trajectory retiming did not converge")
    profile = {"policy": POLICY, "limit_class": limit_class,
               "limits": dict(limits), "points": points,
               "duration_s": points[-1]["time_s"], "wire_quantization": dict(WIRE_QUANTIZATION),
               "waypoint_corridor_rad": WAYPOINT_CORRIDOR_RAD[limit_class],
               "minimum_segment_duration_s": minimum_segment,
               "maximum_source_segment_step_rad": MAX_SOURCE_SEGMENT_STEP_RAD[limit_class]}
    profile["bounds"] = validate_profile(profile)
    profile["timing_method"] = "local-monotone-zero-acceleration"
    return profile


def slice_path(path, start, end):
    """Clip positions AND original times together before compiling a stopped block."""
    if not 0 <= start <= end <= 1:
        raise ValueError("Invalid trajectory progress interval")
    def at(progress):
        if progress <= path[0]["path_progress"]:
            return dict(path[0])
        for a, b in zip(path, path[1:]):
            if b["path_progress"] >= progress:
                span = b["path_progress"] - a["path_progress"]
                u = (progress - a["path_progress"])/span if span > 0 else 0.0
                return {"time_s": a["time_s"] + u*(b["time_s"]-a["time_s"]),
                        "path_progress": progress,
                        "positions": [x+u*(y-x) for x,y in zip(a["positions"],b["positions"])]}
        return dict(path[-1])
    first, last = at(start), at(end)
    if end-start <= 1e-12:
        return [{"time_s": 0.0, "path_progress": start, "positions": first["positions"]},
                {"time_s": MINIMUM_SEGMENT_DURATION_S["print"],
                 "path_progress": end, "positions": first["positions"]}]
    return [first] + [dict(p) for p in path if start < p["path_progress"] < end] + [last]


def sample_profile(profile, spacing_rad=0.002):
    """Sample the nominal command curve with bounded per-joint sample spacing."""
    validate_profile(profile)
    yield list(profile["points"][0]["positions"])
    for a,b in zip(profile["points"], profile["points"][1:]):
        curves = [controls(a,b,j) for j in range(6)]
        # Derivative with respect to normalized u, not wall time.
        bound = max(5*abs(y-x) for curve in curves for x,y in zip(curve,curve[1:]))
        divisions = max(2, int(math.ceil(bound/spacing_rad)))
        for i in range(1, divisions+1):
            u = i/divisions
            values = []
            for curve in curves:
                work = list(curve)
                while len(work)>1:
                    work = [(1-u)*x+u*y for x,y in zip(work,work[1:])]
                values.append(work[0])
            yield values


def validate_bundle(manifest):
    if manifest.get("trajectory_policy_version") != POLICY:
        raise ValueError("This package predates smooth execution; prepare a new package")
    bundle = manifest.get("trajectory_profiles", {})
    continuous = bundle.get("print")
    if continuous is not None:
        approach = bundle.get("approach", {})
        validate_profile(approach, expected_limit_class="approach")
        validate_profile(continuous, expected_limit_class="print")
        if max(
            abs(a-b)
            for a,b in zip(
                approach["points"][-1]["positions"],
                continuous["points"][0]["positions"],
            )
        ) > 1e-8:
            raise ValueError("Approach and continuous print profiles are disconnected")
        marker_times = {
            marker: point["time_s"]
            for point in continuous["points"]
            for marker in point.get("timeline_markers", [])
        }
        previous_time = 0.0
        for index, block in enumerate(manifest.get("blocks", [])):
            start_marker = f"block:{index}:start"
            end_marker = f"block:{index}:end"
            if start_marker not in marker_times or end_marker not in marker_times:
                raise ValueError("Continuous profile is missing a process timeline marker")
            start = _number(block["start_time_s"])
            end = _number(block["end_time_s"])
            duration = _number(block["duration_s"])
            if (
                abs(start-marker_times[start_marker]) > 1e-8
                or abs(end-marker_times[end_marker]) > 1e-8
                or abs(start-previous_time) > 1e-8
                or end <= start
                or abs(end-start-duration) > 1e-8
            ):
                raise ValueError("Continuous process timing differs from the UR profile")
            previous_time = end
        if abs(previous_time-continuous["duration_s"]) > 1e-8:
            raise ValueError("Continuous process timing does not cover the print profile")
        return bundle
    profiles = bundle.get("blocks", [])
    blocks = manifest.get("blocks", [])
    if not profiles or len(profiles) != len(blocks):
        raise ValueError("Smooth profiles do not match the execution blocks")
    approach = bundle.get("approach", {})
    validate_profile(approach, expected_limit_class="approach")
    previous = approach["points"][-1]["positions"]
    for block, profile in zip(blocks, profiles):
        validate_profile(profile, expected_limit_class="print")
        block_duration = _number(block["duration_s"])
        if block_duration <= 0.0 or abs(profile["duration_s"] - block_duration) > 1e-8:
            raise ValueError("UR/Klipper block durations differ")
        if max(abs(a-b) for a,b in zip(previous, profile["points"][0]["positions"])) > 1e-8:
            raise ValueError("Smooth execution profiles are disconnected")
        previous = profile["points"][-1]["positions"]
    return bundle
