#pragma once

#include <cmath>
#include <stdexcept>

namespace viewpoint_manager_ns {
// Cheap prediction geometry. Actual observation uses LiDARModel's full pose
// and requires a measured ray; a prediction is never evidence of coverage.
class ForwardView {
 public:
  ForwardView(double horizontal_deg, double vertical_deg, double range)
      : range_squared_(range * range),
        cos_half_horizontal_(std::cos(horizontal_deg * M_PI / 360.)),
        tan_half_vertical_squared_(std::pow(std::tan(vertical_deg * M_PI / 360.), 2)) {
    if (!std::isfinite(horizontal_deg) || horizontal_deg <= 0 || horizontal_deg > 360 ||
        !std::isfinite(vertical_deg) || vertical_deg <= 0 || vertical_deg >= 180 ||
        !std::isfinite(range) || range <= 0)
      throw std::invalid_argument("Invalid forward prediction FOV/range");
  }
  bool InVerticalRange(double dx, double dy, double dz) const {
    const double xy2 = dx * dx + dy * dy;
    const double r2 = xy2 + dz * dz;
    return std::isfinite(r2) && r2 > 1e-8 && r2 <= range_squared_ + 1e-9 &&
           dz * dz <= xy2 * tan_half_vertical_squared_ + 1e-9;
  }
  bool Contains(double dx, double dy, double dz, double heading_x, double heading_y) const {
    if (!InVerticalRange(dx, dy, dz)) return false;
    return dx * heading_x + dy * heading_y >=
           std::hypot(dx, dy) * cos_half_horizontal_ - 1e-9;
  }
 private:
  double range_squared_, cos_half_horizontal_, tan_half_vertical_squared_;
};
}  // namespace viewpoint_manager_ns
