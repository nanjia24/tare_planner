#pragma once

#include <cmath>

namespace viewpoint_manager_ns
{
inline double ConservativeAssociationRadiusXY(
    double envelope_radius, double cell_resolution_x, double cell_resolution_y)
{
  const double half_cell_diagonal =
      0.5 * std::hypot(cell_resolution_x, cell_resolution_y);
  return envelope_radius + half_cell_diagonal;
}

inline bool WithinHorizontalEnvelope(double point_x, double point_y,
                                     double viewpoint_x, double viewpoint_y,
                                     double envelope_radius)
{
  const double dx = point_x - viewpoint_x;
  const double dy = point_y - viewpoint_y;
  return dx * dx + dy * dy <= envelope_radius * envelope_radius;
}

inline bool DiagonalTransitionIsClear(int delta_x, int delta_y,
                                      bool x_side_clear, bool y_side_clear)
{
  if (std::abs(delta_x) != 1 || std::abs(delta_y) != 1)
  {
    return true;
  }
  return x_side_clear && y_side_clear;
}
}  // namespace viewpoint_manager_ns
