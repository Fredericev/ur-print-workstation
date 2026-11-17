#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "trajectory_safety.hpp"

namespace ur3e_print_planner
{

using Quaternion = std::array<double, 4>;  // x, y, z, w

// A start-state shortcut is deliberately much tighter than the normal
// Cartesian planning resolution.  It only removes an overhead round trip when
// the live TCP and the direct IK endpoint are already the reviewed print start.
constexpr double kNearStartTranslationToleranceM = 0.0001;
constexpr double kNearStartOrientationToleranceRad = 0.001;
constexpr double kNearStartJointToleranceRad = 0.001;

// The alternate approach is still Cartesian and starts from the exact live
// joint state.  These bounds keep its lift local to the reviewed workcell;
// they are not a substitute for scene collision and interpolation checks.
constexpr double kDecomposedLiftClearanceM = 0.06;
constexpr double kMaximumDecomposedLiftM = 0.18;
constexpr double kMaximumDecomposedHeightAboveTableM = 0.50;

inline double decomposedApproachHeight(
  double current_tool_z, double staging_tool_z, double bed_plane_z)
{
  if (!std::isfinite(current_tool_z) || !std::isfinite(staging_tool_z) ||
    !std::isfinite(bed_plane_z))
  {
    throw std::runtime_error("non-finite decomposed approach geometry");
  }
  const double height = std::max(current_tool_z, staging_tool_z) +
    kDecomposedLiftClearanceM;
  if (height - current_tool_z > kMaximumDecomposedLiftM ||
    height - bed_plane_z > kMaximumDecomposedHeightAboveTableM)
  {
    throw std::runtime_error("decomposed approach exceeds its bounded lift envelope");
  }
  return height;
}

inline double quaternionAngularDistance(const Quaternion & a, const Quaternion & b)
{
  const double a_norm = std::sqrt(
    a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
  const double b_norm = std::sqrt(
    b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3]);
  if (!std::isfinite(a_norm) || !std::isfinite(b_norm) || a_norm <= 0.0 || b_norm <= 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::abs(
    (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]) /
    (a_norm * b_norm));
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

inline bool canUseNearZeroApproach(
  double translation_error_m,
  const Quaternion & current_orientation,
  const Quaternion & print_orientation,
  const JointPath & direct_path)
{
  if (!std::isfinite(translation_error_m) || translation_error_m < 0.0 ||
    translation_error_m > kNearStartTranslationToleranceM || direct_path.size() < 2)
  {
    return false;
  }
  if (quaternionAngularDistance(current_orientation, print_orientation) >
    kNearStartOrientationToleranceRad)
  {
    return false;
  }
  const auto metrics = measureJointPath(direct_path);
  return std::all_of(metrics.travel.begin(), metrics.travel.end(), [](double travel) {
      return travel <= kNearStartJointToleranceRad;
    });
}

}  // namespace ur3e_print_planner
