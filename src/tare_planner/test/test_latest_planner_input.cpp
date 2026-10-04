#include <gtest/gtest.h>
#include <sensor_coverage_planner/latest_planner_input.h>
#include <thread>
using sensor_coverage_planner_3d_ns::LatestPlannerInput;
TEST(LatestPlannerInput, KeepsNewestInsteadOfQueuingOldWork) {
  LatestPlannerInput<int> input;
  for(int i=1;i<=100;++i) ASSERT_TRUE(input.Push(std::make_shared<int>(i),i));
  ASSERT_EQ(input.Replaced(),99U);
  auto value=input.Take();ASSERT_TRUE(value);EXPECT_EQ(*value,100);
  EXPECT_FALSE(input.Take());
}
TEST(LatestPlannerInput, RejectsDuplicateOutOfOrderAndInvalidMessages) {
  LatestPlannerInput<int> input;
  EXPECT_FALSE(input.Push(nullptr,1));
  EXPECT_FALSE(input.Push(std::make_shared<int>(0),0));
  ASSERT_TRUE(input.Push(std::make_shared<int>(10),10));
  EXPECT_FALSE(input.Push(std::make_shared<int>(9),9));
  EXPECT_EQ(*input.Take(),10);
  EXPECT_FALSE(input.Push(std::make_shared<int>(10),10));
  EXPECT_TRUE(input.Push(std::make_shared<int>(11),11));
  EXPECT_EQ(*input.Take(),11);
}
TEST(LatestPlannerInput, ProducerCanCacheDuringSerializedPlanningWork) {
  LatestPlannerInput<int> input;
  input.Push(std::make_shared<int>(1),1);
  auto working=input.Take();
  std::thread producer([&]{for(int i=2;i<=10000;++i)input.Push(std::make_shared<int>(i),i);});
  producer.join();
  EXPECT_EQ(*working,1); // Immutable snapshot remains valid during computation.
  EXPECT_EQ(*input.Take(),10000);
  EXPECT_EQ(input.Replaced(),9998U);
}
