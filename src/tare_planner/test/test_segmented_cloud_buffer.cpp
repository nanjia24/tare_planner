#include <gtest/gtest.h>
#include "sensor_coverage_planner/segmented_cloud_buffer.h"
using sensor_coverage_planner_3d_ns::SegmentedCloudBuffer;
static SegmentedCloudBuffer::Cloud Cloud(int stamp) {
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp.sec = stamp;
  return cloud;
}
TEST(SegmentedCloudBuffer, IncompleteNewFrameDoesNotReplaceCompletedPair) {
  SegmentedCloudBuffer buffer;
  buffer.Push(Cloud(1), false);
  EXPECT_FALSE(buffer.Take().first);
  buffer.Push(Cloud(1), true);
  buffer.Push(Cloud(2), false);
  auto pair = buffer.Take();
  ASSERT_TRUE(pair.first && pair.second);
  EXPECT_EQ(pair.first->header.stamp.sec, 1);
  EXPECT_EQ(pair.second->header.stamp.sec, 1);
  EXPECT_FALSE(buffer.Take().first);
  buffer.Push(Cloud(2), true);
  EXPECT_EQ(buffer.Take().first->header.stamp.sec, 2);
}
TEST(SegmentedCloudBuffer, KeepsNewestPairAndRejectsReplayedOrEvictedFrames) {
  SegmentedCloudBuffer buffer;
  for (int stamp = 1; stamp <= 3; ++stamp) {
    buffer.Push(Cloud(stamp), true);
    buffer.Push(Cloud(stamp), false);
  }
  EXPECT_EQ(buffer.Take().first->header.stamp.sec, 3);
  buffer.Push(Cloud(2), true);
  buffer.Push(Cloud(2), false);
  EXPECT_FALSE(buffer.Take().first);
  for (int stamp = 4; stamp <= 8; ++stamp) buffer.Push(Cloud(stamp), false);
  buffer.Push(Cloud(4), true);
  EXPECT_FALSE(buffer.Take().first);  // Frame 4 was evicted from the bounded cache.
  buffer.Push(Cloud(8), true);
  auto pair = buffer.Take();
  ASSERT_TRUE(pair.first && pair.second);
  EXPECT_EQ(pair.first->header.stamp.sec, 8);
  EXPECT_EQ(pair.second->header.stamp.sec, 8);
}
