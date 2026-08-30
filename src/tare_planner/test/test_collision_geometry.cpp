#include <gtest/gtest.h>

#include <viewpoint_manager/collision_geometry.h>

namespace
{
TEST(CollisionGeometry, BroadphaseIncludesCellBoundaryCollision)
{
  constexpr double kEnvelopeRadius = 0.4;
  constexpr double kCellResolution = 0.2;
  constexpr double kCellCenterX = 0.0;
  constexpr double kViewpointX = 0.45;
  constexpr double kObstacleX = 0.099;

  const double association_radius = viewpoint_manager_ns::ConservativeAssociationRadiusXY(
      kEnvelopeRadius, kCellResolution, kCellResolution);

  EXPECT_GT(association_radius, kViewpointX - kCellCenterX);
  EXPECT_TRUE(viewpoint_manager_ns::WithinHorizontalEnvelope(
      kObstacleX, 0.0, kViewpointX, 0.0, kEnvelopeRadius));
}

TEST(CollisionGeometry, ExactCheckRejectsBroadphaseOnlyCandidate)
{
  constexpr double kEnvelopeRadius = 0.4;
  constexpr double kCellResolution = 0.2;
  constexpr double kCellCenterX = 0.0;
  constexpr double kViewpointX = 0.5;
  constexpr double kObstacleX = -0.099;

  const double association_radius = viewpoint_manager_ns::ConservativeAssociationRadiusXY(
      kEnvelopeRadius, kCellResolution, kCellResolution);

  EXPECT_GT(association_radius, kViewpointX - kCellCenterX);
  EXPECT_FALSE(viewpoint_manager_ns::WithinHorizontalEnvelope(
      kObstacleX, 0.0, kViewpointX, 0.0, kEnvelopeRadius));
}

TEST(CollisionGeometry, DiagonalTransitionRequiresBothSideCells)
{
  EXPECT_TRUE(viewpoint_manager_ns::DiagonalTransitionIsClear(1, 0, false, false));
  EXPECT_TRUE(viewpoint_manager_ns::DiagonalTransitionIsClear(1, 1, true, true));
  EXPECT_FALSE(viewpoint_manager_ns::DiagonalTransitionIsClear(1, 1, false, true));
  EXPECT_FALSE(viewpoint_manager_ns::DiagonalTransitionIsClear(-1, 1, true, false));
}
}  // namespace
