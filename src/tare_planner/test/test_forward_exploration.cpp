#include <gtest/gtest.h>
#include <limits>
#include <sensor_coverage_planner/sensor_coverage_planner_ground.h>
#include <viewpoint_manager/forward_view.h>

using sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D;

TEST(ForwardPrediction, RangeVerticalYawAndAzimuthSeam) {
  viewpoint_manager_ns::ForwardView fov(120, 90, 20);
  EXPECT_TRUE(fov.Contains(19, 0, 0, 1, 0));
  EXPECT_TRUE(fov.Contains(20, 0, 0, 1, 0));
  EXPECT_FALSE(fov.Contains(20.01, 0, 0, 1, 0));
  EXPECT_FALSE(fov.Contains(-3, 0, 0, 1, 0));
  EXPECT_FALSE(fov.Contains(1, 2, 0, 1, 0));
  EXPECT_TRUE(fov.Contains(2, 0, 2, 1, 0));
  EXPECT_FALSE(fov.Contains(2, 0, 2.1, 1, 0));
  EXPECT_TRUE(fov.Contains(-3, .001, 0, -1, 0));
  EXPECT_TRUE(fov.Contains(-3, -.001, 0, -1, 0));
  EXPECT_FALSE(fov.Contains(0, 0, 0, 1, 0));
  EXPECT_FALSE(fov.Contains(std::numeric_limits<double>::quiet_NaN(), 0, 0, 1, 0));
  EXPECT_THROW(viewpoint_manager_ns::ForwardView(120, 180, 20), std::invalid_argument);
}

TEST(ForwardPrediction, OcclusionCacheSurvivesHeadingChangesAndSupportsWideVerticalFov) {
  lidar_model_ns::LiDARModel model;
  model.ConfigurePrediction(90);
  // Unknown ray is prospective gain, never proof of actual observation.
  EXPECT_TRUE(model.CheckVisibility(pcl::PointXYZ(3, 0, 0), .1));
  model.UpdateCoverage(pcl::PointXYZ(2, 0, 1));
  EXPECT_TRUE(model.CheckVisibility(pcl::PointXYZ(2, 0, 1), .1));
  EXPECT_FALSE(model.CheckVisibility(pcl::PointXYZ(6, 0, 3), .1));
  model.UpdateCoverage(pcl::PointXYZ(-2, 0, 0));
  EXPECT_FALSE(model.CheckVisibility(pcl::PointXYZ(-6, 0, 0), .1));
  model.ConfigureForwardSensor(120, 90, 20);
  EXPECT_FALSE(model.CheckVisibility(pcl::PointXYZ(3, 0, 0), .1));
}

namespace viewpoint_manager_ns {
struct ForwardExplorationRegressionPeer {
  static void Check(ViewPointManager& manager) {
    manager.UpdateRobotPosition(Eigen::Vector3d::Zero());
    for (auto& viewpoint : manager.viewpoints_) {
      viewpoint.SetInCollision(false);
      viewpoint.SetInLineOfSight(true);
      viewpoint.SetCandidate(true);
    }
    manager.SetRobotYaw(M_PI / 2);
    manager.CheckViewPointConnectivity();
    const int robot_index = manager.GetViewPointInd(Eigen::Vector3d::Zero());
    const int array_index = manager.grid_->GetArrayInd(robot_index);
    EXPECT_TRUE(manager.arrival_directions_[array_index].isApprox(Eigen::Vector2d::UnitY()));
    manager.UpdateViewPointVisited(std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()});
    EXPECT_FALSE(manager.ViewPointVisited(robot_index));
    const auto p = manager.GetViewPointPosition(robot_index);
    EXPECT_TRUE(manager.VisibleByViewPoint(pcl::PointXYZ(p.x, p.y+19, p.z), robot_index));
    EXPECT_FALSE(manager.VisibleByViewPoint(pcl::PointXYZ(p.x, p.y-3, p.z), robot_index));
    EXPECT_FALSE(manager.VisibleByViewPoint(pcl::PointXYZ(p.x, p.y+21, p.z), robot_index));
    // A known obstacle behind the current heading must still enter the cache.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(p.x, p.y-2, p.z));
    manager.UpdateViewPointCoverage<pcl::PointXYZ>(cloud);
    manager.SetRobotYaw(-M_PI / 2);
    manager.CheckViewPointConnectivity();
    EXPECT_FALSE(manager.VisibleByViewPoint(pcl::PointXYZ(p.x, p.y-6, p.z), robot_index));
    EXPECT_TRUE(manager.VisibleByViewPoint(pcl::PointXYZ(p.x, p.y-2, p.z), robot_index));
    // The marker uses the same array mapping/yaw as the gain test.
    manager.candidate_indices_ = {robot_index};
    auto poses = manager.GetCandidateObservationPoses();
    ASSERT_EQ(poses.poses.size(), 1u);
    EXPECT_NEAR(poses.poses[0].orientation.z, -std::sqrt(.5), 1e-6);
    // Forward mode off retains position-based visit semantics.
    manager.vp_.kUseForwardSensor = false;
    manager.UpdateViewPointVisited(std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()});
    EXPECT_TRUE(manager.ViewPointVisited(robot_index));
  }
};
}

namespace sensor_coverage_planner_3d_ns {
struct ForwardExplorationRegressionPeer {
  static void Check() {
    auto node = std::make_shared<SensorCoveragePlanner3D>();
    node->ReadParameters();
    node->set_parameters({rclcpp::Parameter("kUseForwardSensor", true),
        rclcpp::Parameter("kSensorRange", 20.),
        rclcpp::Parameter("viewpoint_manager/number_x", 10),
        rclcpp::Parameter("viewpoint_manager/number_y", 10),
        rclcpp::Parameter("viewpoint_manager/number_z", 1)});
    viewpoint_manager_ns::ViewPointManager manager(node);
    viewpoint_manager_ns::ForwardExplorationRegressionPeer::Check(manager);

    node->robot_viewpoint_.ConfigureForwardSensor(120, 90, 20);
    node->registered_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
        node, "/regression/current_cloud", "map");
    pcl::PointXYZI point;
    point.x = 19; point.y = point.z = 0;
    node->registered_cloud_->cloud_->push_back(point);
    // No PlanningEnv is installed: this must not access a historical scene.
    node->UpdateRobotViewPointCoverage();
    EXPECT_TRUE(node->robot_viewpoint_.CheckVisibility(point, .3));
    node->registered_cloud_->cloud_->clear();
    node->robot_viewpoint_.ResetCoverage();
    node->UpdateRobotViewPointCoverage();
    EXPECT_FALSE(node->robot_viewpoint_.CheckVisibility(point, .3));
  }
};
}

TEST(ForwardExploration, RealManagerAndCurrentObservationIntegration) {
  rclcpp::init(0, nullptr);
  sensor_coverage_planner_3d_ns::ForwardExplorationRegressionPeer::Check();
  rclcpp::shutdown();
}
