#!/usr/bin/env python3
"""Read saved approach winding; optionally compare plan-only Cartesian routes.

No actions, controllers, joint-state publishers or planning-scene writes exist
in this diagnostic. Optional --display publishes only RViz DisplayTrajectory.
It uses the recorded start and never overwrites reviews or execution packages.
"""
import argparse
import copy
import csv
import json
import math
from pathlib import Path


def metrics(names, positions):
    result = {}
    for j, name in enumerate(names):
        q = [p[j] for p in positions]
        travel = sum(abs(b-a) for a, b in zip(q, q[1:]))
        delta = q[-1]-q[0]
        result[name] = {
            'start_deg': round(math.degrees(q[0]), 3),
            'end_deg': round(math.degrees(q[-1]), 3),
            'total_travel_deg': round(math.degrees(travel), 3),
            'backtrack_deg': round(math.degrees(travel-abs(delta)), 3),
            'max_sample_step_deg': round(math.degrees(max(
                (abs(b-a) for a, b in zip(q, q[1:])), default=0)), 3),
        }
    return result


def compare_moveit(names, positions, review, display=False):
    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import RobotState, PlanningSceneComponents
    from moveit_msgs.srv import (GetCartesianPath, GetPositionFK,
                                 GetPlanningScene, GetStateValidity)

    rclpy.init()
    node = rclpy.create_node('approach_read_only_diagnostic')
    clients = {}

    def call(kind, name, request):
        if name not in clients:
            clients[name] = node.create_client(kind, name)
            if not clients[name].wait_for_service(timeout_sec=10.0):
                raise RuntimeError('service unavailable: ' + name)
        client = clients[name]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('service timed out: ' + name)
        result = future.result()
        return result

    def state(q):
        s = RobotState()
        s.is_diff = True
        s.joint_state.name = names
        s.joint_state.position = q
        return s

    def fk(q, link='tool0', frame='base_link'):
        req = GetPositionFK.Request()
        req.header.frame_id = frame
        req.fk_link_names = [link]
        req.robot_state = state(q)
        response = call(GetPositionFK, '/compute_fk', req)
        if response.error_code.val != 1:
            raise RuntimeError('FK failed: ' + str(response.error_code.val))
        return response.pose_stamped[0].pose

    def check_interpolation(label, path_names, path):
        invalid = []
        checked = 0
        for index in range(len(path)):
            a = path[max(index-1, 0)]
            b = path[index]
            count = max(1, math.ceil(max(abs(y-x) for x, y in zip(a, b)) /
                                    math.radians(1.0)))
            for k in range(1, count+1):
                q = [x+(y-x)*k/count for x, y in zip(a, b)]
                req = GetStateValidity.Request()
                req.group_name = 'ur_manipulator'
                req.robot_state = state(q)
                req.robot_state.joint_state.name = list(path_names)
                result = call(GetStateValidity, '/check_state_validity', req)
                checked += 1
                if not result.valid:
                    invalid.append({'segment_end': index, 'fraction': k/count,
                        'contacts': [[c.contact_body_1, c.contact_body_2]
                                     for c in result.contacts[:5]]})
                    if len(invalid) >= 3:
                        break
            if len(invalid) >= 3:
                break
        print(json.dumps({'interpolation_check': label, 'max_joint_spacing_deg': 1,
            'checked': checked, 'invalid_examples': invalid,
            'all_samples_valid': not invalid}), flush=True)
        return not invalid

    def display_candidate(solution):
        from moveit_msgs.msg import DisplayTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from rclpy.qos import QoSProfile, DurabilityPolicy
        # Densify the same linear interpolant for legible RViz playback. These
        # are visualization samples, not a controller command or new IK path.
        shown = copy.deepcopy(solution)
        samples = [copy.deepcopy(shown.joint_trajectory.points[0])]
        for a, b in zip(solution.joint_trajectory.points,
                        solution.joint_trajectory.points[1:]):
            count = max(1, math.ceil(max(abs(y-x) for x, y in
                zip(a.positions, b.positions)) / math.radians(.25)))
            t0 = a.time_from_start.sec + a.time_from_start.nanosec * 1e-9
            t1 = b.time_from_start.sec + b.time_from_start.nanosec * 1e-9
            for k in range(1, count+1):
                p = JointTrajectoryPoint()
                p.positions = [x+(y-x)*k/count for x,y in zip(a.positions,b.positions)]
                ns = round((t0+(t1-t0)*k/count)*1e9)
                p.time_from_start.sec, p.time_from_start.nanosec = divmod(ns, 10**9)
                samples.append(p)
        shown.joint_trajectory.points = samples
        msg = DisplayTrajectory()
        # Empty ID avoids claiming that a model name replaces its calibration.
        msg.model_id = ''
        msg.trajectory_start = state(positions[0])
        msg.trajectory = [shown]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(DisplayTrajectory, '/display_planned_path', qos)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=.2)
            publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=.5)
        print(json.dumps({'rviz_display_only': True, 'samples': len(samples),
                          'recorded_start_not_live_execution': True}), flush=True)

    try:
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        scene = call(GetPlanningScene, '/get_planning_scene', req).scene
        guard = next((x for x in scene.world.collision_objects
                      if x.id == 'confirmed_table_surface_guard'), None)
        if guard is None:
            raise RuntimeError('confirmed table guard is missing; no comparison run')
        shape, pose = guard.primitives[0], guard.primitive_poses[0]
        # MoveIt may canonicalize the box into object.pose in world, with an
        # identity primitive pose. Confirm world/base_link equivalence first.
        if guard.header.frame_id != 'base_link':
            base = fk(positions[0], 'base_link', guard.header.frame_id)
            if (sum(abs(getattr(base.position, a)) for a in 'xyz') > 1e-8 or
                    abs(abs(base.orientation.w)-1) > 1e-8):
                raise RuntimeError('nonidentity base transform; explicit conversion needed')
        if any(abs(abs(p.orientation.w)-1) > 1e-8 for p in (guard.pose, pose)):
            raise RuntimeError('rotated guard; explicit conversion needed')
        top = guard.pose.position.z + pose.position.z + shape.dimensions[2] / 2
        center = [getattr(guard.pose.position, a) + getattr(pose.position, a)
                  for a in 'xy']
        if (abs(top + .014) > 1e-6 or shape.dimensions[2] < 2.0 or
                abs(center[0] + .2925) > 1e-6 or abs(center[1]) > 1e-6 or
                abs(shape.dimensions[0]-.45) > 1e-6 or
                abs(shape.dimensions[1]-.54) > 1e-6):
            raise RuntimeError('table guard differs from the confirmed scene')
        print(json.dumps({'table_guard': {'dimensions': list(shape.dimensions),
            'top_z': top}, 'attached_objects': [x.object.id for x in
                scene.robot_state.attached_collision_objects]}), flush=True)

        initial = fk(positions[0])
        goal = Pose()
        target = review['path_start_pose']
        for axis in ('x', 'y', 'z'):
            setattr(goal.position, axis, float(target[axis]))
        for axis in ('x', 'y', 'z', 'w'):
            setattr(goal.orientation, axis, float(target['q'+axis]))
        staging = copy.deepcopy(goal)
        staging.position.z += .12
        rotated = copy.deepcopy(initial)
        rotated.orientation = copy.deepcopy(goal.orientation)
        lift = copy.deepcopy(initial)
        lift.position.z += .08
        print(json.dumps({'recorded_start_fk': {'x': initial.position.x,
            'y': initial.position.y, 'z': initial.position.z}}), flush=True)
        for index in sorted(set([0, len(positions)//4, len(positions)//2,
                                 3*len(positions)//4, len(positions)-1,
                                 min(range(len(positions)), key=lambda i: positions[i][0])])):
            pose = fk(positions[index])
            print(json.dumps({'old_path_sample': index, 'tool0_xyz':
                [pose.position.x, pose.position.y, pose.position.z]}), flush=True)
        check_interpolation('recorded_approach', names, positions)
        for label, waypoints in [
            ('same_start_cartesian_to_overhead_then_down', [staging, goal]),
            ('rotate_at_current_height_then_overhead_then_down',
             [rotated, staging, goal]),
            ('lift_80mm_then_overhead_then_down', [lift, staging, goal]),
        ]:
            req = GetCartesianPath.Request()
            req.header.frame_id = 'base_link'
            req.group_name = 'ur_manipulator'
            req.link_name = 'tool0'
            req.start_state = state(positions[0])
            req.waypoints = waypoints
            req.max_step = .0025
            req.jump_threshold = 0.0
            req.avoid_collisions = True
            response = call(GetCartesianPath, '/compute_cartesian_path', req)
            jt = response.solution.joint_trajectory
            data = {'comparison': label, 'fraction': response.fraction,
                'error_code': response.error_code.val, 'points': len(jt.points)}
            if jt.points:
                data['joints'] = metrics(jt.joint_names,
                    [list(p.positions) for p in jt.points])
            # A fraction alone is not approval: report jumps and partial paths.
            print(json.dumps(data), flush=True)
            if label == 'same_start_cartesian_to_overhead_then_down' and jt.points:
                valid = check_interpolation(label, jt.joint_names,
                    [list(p.positions) for p in jt.points])
                no_large_jump = all(m['max_sample_step_deg'] < math.degrees(.35)
                                    for m in data['joints'].values())
                if display and valid and no_large_jump and response.fraction >= 1-1e-6:
                    display_candidate(response.solution)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('review_prefix', type=Path)
    parser.add_argument('--compare-moveit', action='store_true')
    parser.add_argument('--display', action='store_true',
        help='Display the first candidate in RViz only if comparison checks pass')
    parser.add_argument('--check-backtrack', action='store_true',
        help='Diagnostic heuristic: flag >180 degrees of joint backtracking')
    args = parser.parse_args()
    rows = list(csv.DictReader(args.review_prefix.with_suffix('.approach.csv').open()))
    names = [x for x in rows[0] if x not in ('time_s', 'path_progress')]
    positions = [[float(r[x]) for x in names] for r in rows]
    result = metrics(names, positions)
    print(json.dumps({'recorded_approach': str(args.review_prefix),
        'duration_s': float(rows[-1]['time_s']), 'joints': result}, indent=2), flush=True)
    if args.compare_moveit:
        compare_moveit(names, positions, json.loads(
            args.review_prefix.with_suffix('.json').read_text()), args.display)
    if args.check_backtrack:
        flagged = [n for n, m in result.items() if m['backtrack_deg'] > 180]
        print('FAIL excessive joint backtracking: ' + ', '.join(flagged)
              if flagged else 'PASS backtracking diagnostic')
        return int(bool(flagged))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
