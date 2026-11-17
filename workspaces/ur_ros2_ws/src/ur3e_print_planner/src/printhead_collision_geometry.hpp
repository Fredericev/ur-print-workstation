#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

// CMake generates geometry and its identity from the same JSON used by the
// workstation and executor. This is the represented CAD envelope; its physical
// completeness and the installed mount still require independent verification.
#include "printhead_collision_geometry_data.hpp"

namespace ur3e_print_planner
{
inline void requirePrintheadGeometryIdentity(const std::string & expected)
{
  if (expected != kPrintheadGeometrySha256)
  {
    throw std::runtime_error(
      "printhead geometry identity differs from the compiled planner; rebuild and review again");
  }
}

// The STEP-derived whole-head box must enclose the configured nozzle TCP.
// A changed TCP outside these bounds requires a new measured collision model;
// silently treating the old box as complete would be unsafe.
inline bool tcpInsidePrintheadEnvelope(double x, double y, double z)
{
  constexpr double tolerance = 1.0e-9;
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         std::abs(x - kPrintheadBoxCenterX) <= kPrintheadBoxSizeX / 2.0 + tolerance &&
         std::abs(y - kPrintheadBoxCenterY) <= kPrintheadBoxSizeY / 2.0 + tolerance &&
         std::abs(z - kPrintheadBoxCenterZ) <= kPrintheadBoxSizeZ / 2.0 + tolerance;
}
}  // namespace ur3e_print_planner
