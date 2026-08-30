#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <sensor_coverage_planner/execution_reference_shortcut.h>

namespace {
using sensor_coverage_planner_3d_ns::BuildOrderedExecutionReferenceArc;
using sensor_coverage_planner_3d_ns::PathTurnSum;
using sensor_coverage_planner_3d_ns::SelectGlobalExecutionReference;
using sensor_coverage_planner_3d_ns::SimplifyExecutionReference;

Eigen::Vector3d Point(double x, double y, double z = 0.2) {
  return Eigen::Vector3d(x, y, z);
}

TEST(ExecutionReferenceShortcut, RemovesSafeShortZigzag) {
  const std::vector<Eigen::Vector3d> points{
      Point(0.0, 0.0), Point(0.35, 0.15), Point(0.70, -0.10), Point(1.05, 0.0)};

  const auto result = SimplifyExecutionReference(
      points, 1.2, 0.1, true, [](const Eigen::Vector3d &) { return true; });

  ASSERT_EQ(result.points.size(), 2U);
  EXPECT_EQ(result.shortcut_count, 1U);
  EXPECT_TRUE(result.points.front().isApprox(points.front()));
  EXPECT_TRUE(result.points.back().isApprox(points.back()));
  EXPECT_LT(PathTurnSum(result.points), PathTurnSum(points));
}

TEST(ExecutionReferenceShortcut, KeepsObstacleDetour) {
  const std::vector<Eigen::Vector3d> points{Point(0.0, 0.0), Point(0.4, 0.5),
                                            Point(0.8, 0.0)};

  const auto result = SimplifyExecutionReference(
      points, 1.2, 0.1, true, [](const Eigen::Vector3d &point) {
        return !(point.x() > 0.25 && point.x() < 0.55 &&
                 std::abs(point.y()) < 0.15);
      });

  ASSERT_EQ(result.points.size(), 3U);
  EXPECT_EQ(result.shortcut_count, 0U);
  EXPECT_TRUE(result.points[1].isApprox(points[1]));
}

TEST(ExecutionReferenceShortcut, DoesNotExceedMaximumChordLength) {
  const std::vector<Eigen::Vector3d> points{Point(0.0, 0.0), Point(0.4, 0.2),
                                            Point(0.8, -0.1), Point(1.3, 0.0)};

  const auto result = SimplifyExecutionReference(
      points, 1.2, 0.1, true, [](const Eigen::Vector3d &) { return true; });

  ASSERT_EQ(result.points.size(), 3U);
  EXPECT_TRUE(result.points[0].isApprox(points[0]));
  EXPECT_TRUE(result.points[1].isApprox(points[2]));
  EXPECT_TRUE(result.points[2].isApprox(points[3]));
}

TEST(ExecutionReferenceShortcut, PreservesSourceOrderAcrossSelfIntersection) {
  const std::vector<Eigen::Vector3d> points{Point(0.0, 0.0), Point(0.5, 0.5),
                                            Point(1.0, 0.0), Point(0.5, -0.5),
                                            Point(0.0, 0.0), Point(-0.5, 0.5)};

  const auto result = SimplifyExecutionReference(
      points, 0.8, 0.1, true, [](const Eigen::Vector3d &) { return true; });

  ASSERT_EQ(result.points.size(), points.size());
  EXPECT_EQ(result.shortcut_count, 0U);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_TRUE(result.points[i].isApprox(points[i]));
  }
}

TEST(ExecutionReferenceShortcut, ReturnsExactInputWhenMapIsUnavailable) {
  const std::vector<Eigen::Vector3d> points{Point(0.0, 0.0), Point(0.4, 0.2),
                                            Point(0.8, 0.0)};

  const auto result = SimplifyExecutionReference(
      points, 1.2, 0.1, false, [](const Eigen::Vector3d &) { return true; });

  ASSERT_EQ(result.points.size(), points.size());
  EXPECT_EQ(result.shortcut_count, 0U);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_TRUE(result.points[i].isApprox(points[i]));
  }
}

TEST(ExecutionReferenceShortcut, PreservesEndpointHeightExactly) {
  const std::vector<Eigen::Vector3d> points{
      Point(0.0, 0.0, 0.18), Point(0.4, 0.1, 0.22), Point(0.8, 0.0, 0.31)};

  const auto result = SimplifyExecutionReference(
      points, 1.2, 0.1, true, [](const Eigen::Vector3d &) { return true; });

  ASSERT_EQ(result.points.size(), 2U);
  EXPECT_DOUBLE_EQ(result.points.front().z(), 0.18);
  EXPECT_DOUBLE_EQ(result.points.back().z(), 0.31);
}

TEST(ExecutionReferenceSelection, UsesVisitIndexAtSelfOverlap) {
  const std::vector<Eigen::Vector3d> path{
      {0.0, 0.0, 0.1}, {1.0, 0.0, 0.1}, {2.0, 0.0, 0.1},
      {1.0, 0.0, 0.1}, {1.0, 1.0, 0.1},
  };

  const auto arc = BuildOrderedExecutionReferenceArc(
      path, Eigen::Vector3d(0.0, 0.0, 0.1), 0, 3, 1, false);

  ASSERT_EQ(arc.size(), 4U);
  EXPECT_TRUE(arc[1].isApprox(Eigen::Vector3d(1.0, 0.0, 0.1)));
  EXPECT_TRUE(arc[2].isApprox(Eigen::Vector3d(2.0, 0.0, 0.1)));
  EXPECT_TRUE(arc[3].isApprox(Eigen::Vector3d(1.0, 0.0, 0.1)));
}

TEST(ExecutionReferenceSelection, SelectsOrderedGlobalFallback) {
  const std::vector<Eigen::Vector3d> path{
      Point(0.0, 0.0), Point(0.5, 0.0), Point(1.0, 0.0),
      Point(1.5, 0.0), Point(0.0, 0.0),
  };

  const auto selection = SelectGlobalExecutionReference(path, 0.8, true);

  ASSERT_TRUE(selection.valid);
  EXPECT_EQ(selection.robot_index, 0);
  EXPECT_EQ(selection.target_index, 2);
  EXPECT_EQ(selection.direction_step, 1);
}
} // namespace
