#pragma once
#include <cstdint>
#include <memory>
#include <mutex>

namespace sensor_coverage_planner_3d_ns {
// Callbacks only retain the newest immutable message. The planning group alone
// consumes it and mutates map/graph/planner state. No unbounded work queue.
template<class Message> class LatestPlannerInput {
 public:
  using Ptr = std::shared_ptr<const Message>;
  bool Push(Ptr message, int64_t stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!message || stamp <= last_stamp_) return false;
    if (pending_) ++replaced_;
    last_stamp_ = stamp;
    pending_ = std::move(message);
    return true;
  }
  Ptr Take() {
    std::lock_guard<std::mutex> lock(mutex_);
    Ptr result;
    result.swap(pending_);
    return result;
  }
  uint64_t Replaced() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return replaced_;
  }
 private:
  mutable std::mutex mutex_;
  Ptr pending_;
  int64_t last_stamp_{0};
  uint64_t replaced_{0};
};
}
