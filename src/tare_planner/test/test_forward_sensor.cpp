#include <gtest/gtest.h>
#include <lidar_model/lidar_model.h>

namespace {
pcl::PointXYZ Point(double x, double y, double z = 0) {
  return pcl::PointXYZ(x, y, z);
}
TEST(ForwardSensor, ObservedRaysAndOcclusion) {
  lidar_model_ns::LiDARModel sensor;
  sensor.ConfigureForwardSensor(120, 90, 10);
  EXPECT_FALSE(sensor.CheckVisibility(Point(3, 0), 0.1));
  sensor.UpdateCoverage(Point(3, 0));
  EXPECT_TRUE(sensor.CheckVisibility(Point(3, 0), 0.1));
  EXPECT_FALSE(sensor.CheckVisibility(Point(4, 0), 0.1));
  EXPECT_FALSE(sensor.CheckVisibility(Point(-3, 0), 0.1));
  sensor.ResetCoverage();
  EXPECT_FALSE(sensor.CheckVisibility(Point(3, 0), 0.1));
}
TEST(ForwardSensor, FieldOfViewRangeAndVerticalCoverage) {
  lidar_model_ns::LiDARModel sensor;
  sensor.ConfigureForwardSensor(120, 90, 10);
  EXPECT_TRUE(sensor.InSensorFOV(Point(2, 3)));
  EXPECT_FALSE(sensor.InSensorFOV(Point(1, 2)));
  EXPECT_TRUE(sensor.InSensorFOV(Point(2, 0, 2)));
  EXPECT_FALSE(sensor.InSensorFOV(Point(2, 0, 3)));
  EXPECT_FALSE(sensor.InSensorFOV(Point(11, 0)));
  sensor.UpdateCoverage(Point(2, 0, 2));
  EXPECT_TRUE(sensor.CheckVisibility(Point(2, 0, 2), 0.1));
}
TEST(ForwardSensor, FullPoseAndWorldAzimuthSeam) {
  lidar_model_ns::LiDARModel sensor;
  sensor.ConfigureForwardSensor(120, 90, 10);
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1;
  pose.orientation.w = 0;
  pose.orientation.z = 1;  // pi yaw
  sensor.setPose(pose);
  sensor.UpdateCoverage(Point(-2, 0));
  EXPECT_TRUE(sensor.CheckVisibility(Point(-2, 0), 0.1));
  EXPECT_FALSE(sensor.InSensorFOV(Point(4, 0)));
  pose.orientation.z = 0;
  pose.orientation.w = std::cos(M_PI / 4);
  pose.orientation.y = std::sin(M_PI / 4);
  sensor.setPose(pose);
  EXPECT_TRUE(sensor.InSensorFOV(Point(1, 0, -3)));
  EXPECT_FALSE(sensor.InSensorFOV(Point(4, 0)));
  pose.orientation.w = pose.orientation.y = 0;
  sensor.setPose(pose);
  EXPECT_FALSE(sensor.InSensorFOV(Point(1, 0, -3)));
}
TEST(ForwardSensor, InPlaceRotationRevealsRearOnlyAfterObservation) {
  lidar_model_ns::LiDARModel sensor;
  sensor.ConfigureForwardSensor(120, 90, 10);
  sensor.UpdateCoverage(Point(3, 0));
  EXPECT_TRUE(sensor.CheckVisibility(Point(3, 0), 0.1));
  EXPECT_FALSE(sensor.CheckVisibility(Point(-3, 0), 0.1));
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 0;
  pose.orientation.z = 1;
  sensor.setPose(pose);
  sensor.ResetCoverage();
  EXPECT_FALSE(sensor.CheckVisibility(Point(-3, 0), 0.1));
  sensor.UpdateCoverage(Point(-3, 0));
  EXPECT_TRUE(sensor.CheckVisibility(Point(-3, 0), 0.1));
  EXPECT_FALSE(sensor.CheckVisibility(Point(3, 0), 0.1));
}
TEST(ForwardSensor, LegacyAndInvalidConfiguration) {
  lidar_model_ns::LiDARModel sensor;
  EXPECT_FALSE(sensor.IsDirectional());
  EXPECT_TRUE(sensor.CheckVisibility(Point(-3, 0), 0.1));
  EXPECT_THROW(sensor.ConfigureForwardSensor(0, 90, 10), std::invalid_argument);
  EXPECT_THROW(sensor.ConfigureForwardSensor(120, 180, 10), std::invalid_argument);
  EXPECT_THROW(sensor.ConfigureForwardSensor(120, 90, -1), std::invalid_argument);
}
}
