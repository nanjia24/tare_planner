#include <gtest/gtest.h>
#include <planning_env/planning_env.h>

TEST(SegmentedCollision, UsesOnlyReceivedObstacleCloudForGraphAndViewpoints)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", TARE_TEST_PARAMETERS});
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("tare_planner_node", options);
  {
    planning_env_ns::PlanningEnv env(node, "camera_init");
    env.UseExternalCollisionCloud(true);
    EXPECT_TRUE(env.InCollision(0, 0, 0));  // Missing segmentation is not free.
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZI>());
    env.SetExternalCollisionCloud(obstacles);
    EXPECT_FALSE(env.InCollision(0, 0, 0));  // Received, genuinely empty nonground.
    for (int i = 0; i < 20; ++i) {
      pcl::PointXYZI point;
      point.x = 1.0 + i * 0.001; point.y = 0; point.z = 0; point.intensity = 1;
      obstacles->push_back(point);
    }
    env.SetExternalCollisionCloud(obstacles);
    env.SetUseFrontier(false);
    geometry_msgs::msg::Point origin;
    env.UpdateRobotPosition(origin);
    pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (int x = -5; x <= 5; ++x) {
      for (int y = -5; y <= 5; ++y) {
        pcl::PointXYZI floor;
        floor.x = x * 0.1; floor.y = y * 0.1; floor.z = 0; floor.intensity = 0;
        full_cloud->push_back(floor);
      }
    }
    env.UpdateKeyposeCloud<pcl::PointXYZI>(full_cloud);  // Full observations cannot replace obstacles.
    EXPECT_EQ(env.GetCollisionCloud()->size(), 20u);
    EXPECT_TRUE(env.InCollision(1, 0, 0));
    EXPECT_FALSE(env.InCollision(5, 0, 0));
    obstacles->clear();  // Caller mutation cannot change the installed KD tree.
    EXPECT_TRUE(env.InCollision(1, 0, 0));
    env.SetExternalCollisionCloud(obstacles);
    EXPECT_FALSE(env.InCollision(1, 0, 0));
  }
  node.reset();
  rclcpp::shutdown();
}
