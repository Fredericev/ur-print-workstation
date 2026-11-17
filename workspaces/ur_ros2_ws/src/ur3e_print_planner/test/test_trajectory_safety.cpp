#include "../include/trajectory_safety.hpp"
#include "../include/approach_policy.hpp"

#include <iostream>
#include <limits>
#include <string>

using namespace ur3e_print_planner;
int failures = 0;

template<typename F>
void expectRejected(const std::string & name, F operation)
{
  try
  {
    operation();
    std::cerr << "FAIL " << name << ": accepted invalid path\n";
    ++failures;
  }
  catch (const std::runtime_error &)
  {
    std::cout << "PASS " << name << '\n';
  }
}

int main()
{
  try
  {
    const double height = decomposedApproachHeight(0.2343998, 0.255725, -0.014);
    if (std::abs(height - 0.315725) > 1.0e-9)
    {
      throw std::runtime_error("unexpected decomposed lift height");
    }
    std::cout << "PASS bounded decomposed lift height\n";
  }
  catch (const std::runtime_error & e)
  {
    std::cerr << "FAIL bounded decomposed lift height: " << e.what() << '\n';
    ++failures;
  }
  expectRejected("decomposed lift too far from live tool", []() {
    decomposedApproachHeight(0.05, 0.25, -0.014);
  });
  expectRejected("decomposed lift above bounded workcell", []() {
    decomposedApproachHeight(0.50, 0.50, -0.014);
  });
  expectRejected("decomposed lift non-finite geometry", []() {
    decomposedApproachHeight(0.20, std::numeric_limits<double>::quiet_NaN(), -0.014);
  });
  const int failures_before_start_policy = failures;
  // Regression from review 36432a037235: the TCP was already 0.025 mm from
  // the first print pose and the largest endpoint joint correction was below
  // 0.0001 rad.  The planner must use the tiny direct correction instead of a
  // 12 cm overhead lift and return.
  const JointPath already_at_start{
    {0.568629920, -2.550241610, -2.350734711, 0.190291329, 1.569073319, 2.143928528},
    {0.568623787, -2.550301177, -2.350717028, 0.190210509, 1.569070810, 2.143949581},
  };
  const Quaternion current_orientation{0.999999998, 0.000013580, -0.000050920, 0.000034243};
  const Quaternion print_orientation{1.0, 0.0, 0.0, 0.0};
  if (!canUseNearZeroApproach(
      0.000024516, current_orientation, print_orientation, already_at_start))
  {
    std::cerr << "FAIL already-at-start approach was not selected\n";
    ++failures;
  }
  if (canUseNearZeroApproach(
      0.02, current_orientation, print_orientation, already_at_start))
  {
    std::cerr << "FAIL genuine Cartesian move bypassed the overhead approach\n";
    ++failures;
  }
  if (canUseNearZeroApproach(
      0.000024516, Quaternion{0.0, 0.0, 0.0, 1.0},
      print_orientation, already_at_start))
  {
    std::cerr << "FAIL wrong tool orientation bypassed the overhead approach\n";
    ++failures;
  }
  auto different_branch = already_at_start;
  different_branch.back()[1] += 0.01;
  if (canUseNearZeroApproach(
      0.000024516, current_orientation, print_orientation, different_branch))
  {
    std::cerr << "FAIL different IK branch bypassed the overhead approach\n";
    ++failures;
  }
  if (failures == failures_before_start_policy)
  {
    std::cout << "PASS already-at-start uses a near-zero direct approach\n";
  }

  // Small adjacent steps can conceal a whole extra revolution/backtracking.
  JointPath winding;
  for (int i = 0; i <= 60; ++i) {winding.push_back({-0.1 * i});}
  for (int i = 59; i >= -2; --i) {winding.push_back({-0.1 * i});}
  expectRejected("small-step large winding", [&]() {
    requireContinuousPath(winding, true);
  });
  JointPath backtrack;
  for (int i = 0; i <= 12; ++i) {backtrack.push_back({0.1 * i});}
  for (int i = 11; i >= 0; --i) {backtrack.push_back({0.1 * i});}
  expectRejected("unnecessary approach backtracking", [&]() {
    requireContinuousPath(backtrack, true);
  });
  expectRejected("collision between valid endpoints", []() {
    validateLinearInterpolation({{0.0}, {0.03}}, [](const auto & q) {
      return std::abs(q[0] - 0.015) > 0.001;
    });
  });
  expectRejected("raw angles must not wrap modulo 2pi", []() {
    requireContinuousPath({{3.1}, {-3.1}}, true);
  });
  expectRejected("nonfinite position", []() {
    requireContinuousPath({{0.0}, {std::numeric_limits<double>::quiet_NaN()}}, true);
  });
  expectRejected("inconsistent joint count", []() {
    requireContinuousPath({{0.0}, {0.0, 0.0}}, true);
  });
  try
  {
    JointPath nearby;
    for (int i = 0; i <= 10; ++i) {nearby.push_back({0.013 * i, 0.149 * i});}
    requireContinuousPath(nearby, true);
    // Printing may legitimately backtrack many times: only the approach is
    // subject to a total-travel budget, never the full printed shape.
    requireContinuousPath(winding, false);
    double previous = 0.0;
    const auto count = validateLinearInterpolation({{0.0}, {0.03}}, [&](const auto & q) {
      const bool within_spacing = q[0] - previous <= kCollisionSampleSpacingRad + 1e-12;
      previous = q[0];
      return within_spacing;
    });
    if (count != 3) {throw std::runtime_error("did not check start/interior/end");}
    std::cout << "PASS nearby approach, legitimate print and dense sampling\n";
  }
  catch (const std::runtime_error & e)
  {
    std::cerr << "FAIL valid-path controls: " << e.what() << '\n';
    ++failures;
  }
  return failures ? 1 : 0;
}
