#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "robot_control_pkg/hw_definition.hpp"
#include "robot_control_pkg/surgical_tool.hpp"

TEST(SurgicalToolFk, StraightLayoutHasNineteenSegments)
{
  SurgicalTool tool;
  const std::vector<double> pan_angles(NUM_OF_BENDING_JOINTS, 0.0);
  const std::vector<double> tilt_angles(NUM_OF_BENDING_JOINTS, 0.0);

  const auto transforms =
    tool.computeBaseToJointsTransformationMatrices(
    pan_angles, tilt_angles);
  const auto joint_positions = tool.computeJointPositions(transforms);
  const auto tip_position = tool.computeEndEffectorPosition(transforms);

  ASSERT_EQ(transforms.size(), 18U);
  ASSERT_EQ(joint_positions.size(), 18U);
  EXPECT_NEAR(joint_positions.front().x(), 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions.back().x(), 18.0 * 4.33e-3, 1e-12);
  EXPECT_NEAR(tip_position.x(), 19.0 * 4.33e-3, 1e-12);
  EXPECT_NEAR(tip_position.y(), 0.0, 1e-12);
  EXPECT_NEAR(tip_position.z(), 0.0, 1e-12);
}

TEST(SurgicalToolFk, FirstPanJointActsAfterFixedProximalSegment)
{
  SurgicalTool tool;
  std::vector<double> pan_angles(NUM_OF_BENDING_JOINTS, 0.0);
  const std::vector<double> tilt_angles(NUM_OF_BENDING_JOINTS, 0.0);
  pan_angles[0] = std::acos(-1.0) / 2.0;

  const auto transforms =
    tool.computeBaseToJointsTransformationMatrices(
    pan_angles, tilt_angles);
  const auto joint_positions = tool.computeJointPositions(transforms);

  EXPECT_NEAR(joint_positions[0].x(), 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions[0].y(), 0.0, 1e-12);
  EXPECT_NEAR(joint_positions[1].x(), 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions[1].y(), 4.33e-3, 1e-12);
}
