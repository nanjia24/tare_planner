#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <utility>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensor_coverage_planner_3d_ns {
// Only immutable message pointers cross executor groups. Planning/map objects
// remain exclusively owned by the default mutually-exclusive callback group.
class SegmentedCloudBuffer {
 public:
  using Cloud = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  using Pair = std::pair<Cloud, Cloud>;  // terrain, ground
  void Push(Cloud cloud, bool ground) {
    const int64_t stamp = int64_t(cloud->header.stamp.sec) * 1000000000LL + cloud->header.stamp.nanosec;
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp <= newest_complete_) return;
    auto& pair = pending_[stamp];
    (ground ? pair.second : pair.first) = std::move(cloud);
    if (pair.first && pair.second) {
      ready_ = pair;
      newest_complete_ = stamp;
      pending_.erase(pending_.begin(), pending_.upper_bound(stamp));
    }
    while (pending_.size() > 4) pending_.erase(pending_.begin());
  }
  Pair Take() {
    std::lock_guard<std::mutex> lock(mutex_);
    Pair result;
    result.swap(ready_);
    return result;
  }
 private:
  std::mutex mutex_;
  int64_t newest_complete_ = 0;
  std::map<int64_t, Pair> pending_;
  Pair ready_;
};
}  // namespace sensor_coverage_planner_3d_ns
