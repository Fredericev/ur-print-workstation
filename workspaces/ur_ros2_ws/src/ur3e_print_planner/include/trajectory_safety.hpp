#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace ur3e_print_planner
{
using JointPath = std::vector<std::vector<double>>;
constexpr double kMaximumJointStepRad = 0.35;
constexpr double kCollisionSampleSpacingRad = 0.017453292519943295;
constexpr double kMaximumApproachTravelRad = 3.14159265358979323846;
constexpr double kMaximumApproachBacktrackRad = 1.57079632679489661923;

struct JointPathMetrics
{
  double max_step = 0.0;
  std::vector<double> travel;
  std::vector<double> backtrack;
};

inline JointPathMetrics measureJointPath(const JointPath & path)
{
  if (path.size() < 2 || path.front().empty())
  {
    throw std::runtime_error("trajectory needs at least two nonempty joint states");
  }
  JointPathMetrics result;
  result.travel.resize(path.front().size(), 0.0);
  result.backtrack.resize(path.front().size(), 0.0);
  for (size_t i = 0; i < path.size(); ++i)
  {
    if (path[i].size() != path.front().size())
    {
      throw std::runtime_error("inconsistent trajectory joint dimensions");
    }
    for (size_t j = 0; j < path[i].size(); ++j)
    {
      if (!std::isfinite(path[i][j]))
      {
        throw std::runtime_error("non-finite trajectory joint position");
      }
      if (i > 0)
      {
        const double step = std::abs(path[i][j] - path[i - 1][j]);
        result.travel[j] += step;
        result.max_step = std::max(result.max_step, step);
      }
    }
  }
  for (size_t j = 0; j < result.travel.size(); ++j)
  {
    result.backtrack[j] = std::max(
      0.0, result.travel[j] - std::abs(path.back()[j] - path.front()[j]));
  }
  return result;
}

inline JointPathMetrics requireContinuousPath(const JointPath & path, bool approach)
{
  const auto result = measureJointPath(path);
  if (result.max_step > kMaximumJointStepRad)
  {
    throw std::runtime_error("excessive raw joint-space step");
  }
  if (approach)
  {
    for (size_t j = 0; j < result.travel.size(); ++j)
    {
      if (result.travel[j] > kMaximumApproachTravelRad ||
        result.backtrack[j] > kMaximumApproachBacktrackRad)
      {
        throw std::runtime_error("excessive approach joint travel or backtracking");
      }
    }
  }
  return result;
}

template<typename StateIsValid>
size_t validateLinearInterpolation(const JointPath & path, StateIsValid valid)
{
  measureJointPath(path);
  size_t checked = 0;
  for (size_t i = 0; i < path.size(); ++i)
  {
    const auto & a = path[i == 0 ? 0 : i - 1];
    const auto & b = path[i];
    double max_delta = 0.0;
    for (size_t j = 0; j < a.size(); ++j)
    {
      max_delta = std::max(max_delta, std::abs(b[j] - a[j]));
    }
    const auto steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(max_delta / kCollisionSampleSpacingRad)));
    for (size_t k = 1; k <= steps; ++k)
    {
      std::vector<double> point(a.size());
      for (size_t j = 0; j < a.size(); ++j)
      {
        point[j] = a[j] + (b[j] - a[j]) * static_cast<double>(k) / steps;
      }
      ++checked;
      if (!valid(point))
      {
        throw std::runtime_error("trajectory joint interpolation is invalid at segment " +
          std::to_string(i) + ", sample " + std::to_string(k) + "/" + std::to_string(steps));
      }
    }
  }
  return checked;
}
}  // namespace ur3e_print_planner
