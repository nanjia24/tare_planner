#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

namespace sensor_coverage_planner_3d_ns {
struct ExecutionReferenceShortcutResult {
  std::vector<Eigen::Vector3d> points;
  std::size_t shortcut_count{0};
};

struct OrderedExecutionReferenceSelection {
  int robot_index{-1};
  int target_index{-1};
  int direction_step{0};
  bool valid{false};
};

inline OrderedExecutionReferenceSelection
SelectGlobalExecutionReference(const std::vector<Eigen::Vector3d> &path,
                               double lookahead_distance, bool from_start) {
  if (path.size() < 2 || !std::isfinite(lookahead_distance) ||
      lookahead_distance < 0.0) {
    return {};
  }

  double distance = 0.0;
  if (from_start) {
    for (int index = 1; index < static_cast<int>(path.size()); ++index) {
      distance += (path[index] - path[index - 1]).norm();
      if (distance > lookahead_distance) {
        return {0, index, 1, true};
      }
    }
  } else {
    const int robot_index = static_cast<int>(path.size()) - 1;
    for (int index = robot_index - 1; index > 0; --index) {
      distance += (path[index] - path[index + 1]).norm();
      if (distance > lookahead_distance) {
        return {robot_index, index, -1, true};
      }
    }
  }
  return {};
}

inline std::vector<Eigen::Vector3d>
BuildOrderedExecutionReferenceArc(const std::vector<Eigen::Vector3d> &path,
                                  const Eigen::Vector3d &robot_position,
                                  int robot_index, int target_index,
                                  int direction_step, bool loop) {
  if (path.size() < 2 || robot_index < 0 || target_index < 0 ||
      robot_index >= static_cast<int>(path.size()) ||
      target_index >= static_cast<int>(path.size()) ||
      robot_index == target_index ||
      (direction_step != 1 && direction_step != -1)) {
    return {};
  }

  std::vector<Eigen::Vector3d> arc{robot_position};
  int index = robot_index;
  for (std::size_t count = 0; count < path.size(); ++count) {
    index += direction_step;
    if (loop) {
      if (index < 0) {
        index = static_cast<int>(path.size()) - 1;
      } else if (index >= static_cast<int>(path.size())) {
        index = 0;
      }
    } else if (index < 0 || index >= static_cast<int>(path.size())) {
      return {};
    }

    arc.push_back(path[index]);
    if (index == target_index) {
      return arc;
    }
  }
  return {};
}

inline bool ExecutionReferencePointIsFinite(const Eigen::Vector3d &point) {
  return std::isfinite(point.x()) && std::isfinite(point.y()) &&
         std::isfinite(point.z());
}

inline bool ExecutionReferenceSegmentIsClear(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    double sample_step,
    const std::function<bool(const Eigen::Vector3d &)> &point_valid) {
  const double length = (end - start).norm();
  if (!std::isfinite(length)) {
    return false;
  }

  const int interval_count =
      std::max(1, static_cast<int>(std::ceil(length / sample_step)));
  for (int sample_index = 1; sample_index < interval_count; ++sample_index) {
    const double ratio = static_cast<double>(sample_index) / interval_count;
    if (!point_valid(start + ratio * (end - start))) {
      return false;
    }
  }
  return true;
}

inline ExecutionReferenceShortcutResult SimplifyExecutionReference(
    const std::vector<Eigen::Vector3d> &input, double max_shortcut_distance,
    double sample_step, bool map_ready,
    const std::function<bool(const Eigen::Vector3d &)> &point_valid) {
  ExecutionReferenceShortcutResult result{input, 0};
  if (input.size() < 3 || !map_ready || !point_valid ||
      !std::isfinite(max_shortcut_distance) || max_shortcut_distance <= 0.0 ||
      !std::isfinite(sample_step) || sample_step <= 0.0) {
    return result;
  }
  if (!std::all_of(input.begin(), input.end(),
                   ExecutionReferencePointIsFinite)) {
    return result;
  }

  result.points.clear();
  result.points.reserve(input.size());
  result.points.push_back(input.front());

  std::size_t source_index = 0;
  while (source_index + 1 < input.size()) {
    std::size_t selected_index = source_index + 1;
    double source_arc_length = 0.0;
    for (std::size_t candidate_index = source_index + 1;
         candidate_index < input.size(); ++candidate_index) {
      source_arc_length +=
          (input[candidate_index] - input[candidate_index - 1]).norm();
      if (source_arc_length > max_shortcut_distance + 1e-9) {
        break;
      }
      if (candidate_index <= source_index + 1) {
        continue;
      }

      const double chord_length =
          (input[candidate_index] - input[source_index]).norm();
      if (chord_length > max_shortcut_distance + 1e-9) {
        continue;
      }
      if (ExecutionReferenceSegmentIsClear(input[source_index],
                                           input[candidate_index], sample_step,
                                           point_valid)) {
        selected_index = candidate_index;
      }
    }

    if (selected_index > source_index + 1) {
      ++result.shortcut_count;
    }
    result.points.push_back(input[selected_index]);
    source_index = selected_index;
  }
  return result;
}

inline double PathTurnSum(const std::vector<Eigen::Vector3d> &points) {
  double turn_sum = 0.0;
  for (std::size_t index = 1; index + 1 < points.size(); ++index) {
    const Eigen::Vector2d incoming =
        (points[index] - points[index - 1]).head<2>();
    const Eigen::Vector2d outgoing =
        (points[index + 1] - points[index]).head<2>();
    const double incoming_norm = incoming.norm();
    const double outgoing_norm = outgoing.norm();
    if (incoming_norm <= 1e-9 || outgoing_norm <= 1e-9) {
      continue;
    }
    const double cosine =
        std::max(-1.0, std::min(1.0, incoming.dot(outgoing) /
                                         (incoming_norm * outgoing_norm)));
    turn_sum += std::acos(cosine);
  }
  return turn_sum;
}
} // namespace sensor_coverage_planner_3d_ns
