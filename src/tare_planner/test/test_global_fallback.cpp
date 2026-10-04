#include <gtest/gtest.h>
#include <sensor_coverage_planner/sensor_coverage_planner_ground.h>
#include <thread>
#include <limits>

namespace sensor_coverage_planner_3d_ns {
struct GlobalFallbackRegressionPeer {
  static void Check() {
    auto node = std::make_shared<SensorCoveragePlanner3D>();
    // Exhaustion plus a parking fallback cannot finish a blocked mission.
    node->at_home_ = false;
    node->UpdateCompletionState(true, false, false);
    EXPECT_FALSE(node->exploration_finished_);
    node->at_home_ = true;
    node->UpdateCompletionState(true, true, false);
    EXPECT_FALSE(node->exploration_finished_);
    EXPECT_FALSE(node->stopped_);
    node->UpdateCompletionState(true, false, false);
    EXPECT_TRUE(node->exploration_finished_);
    EXPECT_TRUE(node->stopped_);
    node->exploration_finished_ = node->stopped_ = false;
    node->at_home_ = false;
    node->UpdateCompletionState(true, false, true);
    EXPECT_TRUE(node->exploration_finished_);
    EXPECT_FALSE(node->stopped_); // Returning home is still an active task.
    node->robot_position_.x = 0.;
    node->robot_position_.y = 0.;
    node->robot_position_.z = .16;
    node->kLookAheadDistance = 4.;
    exploration_path_ns::ExplorationPath local, global;
    local.nodes_.emplace_back(Eigen::Vector3d(1.2, 0., .16));
    // Reproduces the distant singleton that escaped the short-path gate.
    global.nodes_.emplace_back(Eigen::Vector3d(0., 0., .16));
    global.nodes_.emplace_back(Eigen::Vector3d(3., 0., .16));
    global.nodes_.back().type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
    global.nodes_.emplace_back(Eigen::Vector3d(10., 0., .16));
    Eigen::Vector3d lookahead(99., 99., 99.);
    node->GetLookAheadPoint(local, global, lookahead);
    EXPECT_TRUE(node->execution_reference_selection_valid_);
    EXPECT_TRUE(node->execution_reference_uses_global_path_);
    EXPECT_TRUE(lookahead.isApprox(Eigen::Vector3d(3., 0., .16)));
    exploration_path_ns::ExplorationPath empty_local;
    const auto concatenated = node->ConcatenateGlobalLocalPath(global, empty_local);
    EXPECT_EQ(concatenated.GetNodeNum(), 0);
    node->GetLookAheadPoint(concatenated, global, lookahead);
    EXPECT_TRUE(node->execution_reference_uses_global_path_);
    global.nodes_.clear();
    node->GetLookAheadPoint(local, global, lookahead);
    EXPECT_FALSE(node->execution_reference_selection_valid_);
    EXPECT_TRUE(lookahead.isApprox(Eigen::Vector3d(0., 0., .16)));

    node->lookahead_point_ = lookahead;
    node->kExtendWayPoint = true;
    node->kExtendWayPointDistanceBig = 4.;
    node->kExtendWayPointDistanceSmall = 2.;
    node->waypoint_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("/regression/waypoint", 10);
    std::vector<geometry_msgs::msg::PointStamped> received;
    auto subscription = node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/regression/waypoint", 10, [&](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) { received.push_back(*msg); });
    const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while (received.empty() && std::chrono::steady_clock::now()<deadline) {
      node->PublishWaypoint();
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_FALSE(received.empty());
    EXPECT_DOUBLE_EQ(received.back().point.x, 0.);
    EXPECT_DOUBLE_EQ(received.back().point.y, 0.);
    EXPECT_DOUBLE_EQ(received.back().point.z, .16);
  }
};
}
TEST(GlobalFallback, DistantSingletonUsesGlobalAndEmptyGlobalStopsWithFiniteWaypoint) {
  rclcpp::init(0, nullptr);
  sensor_coverage_planner_3d_ns::GlobalFallbackRegressionPeer::Check();
  rclcpp::shutdown();
}
