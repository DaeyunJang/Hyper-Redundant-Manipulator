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
  const bool first_frame_is_base_aligned =
    transforms.front().block<3, 3>(0, 0).isApprox(
      Eigen::Matrix3d::Identity(), 1e-12);
  EXPECT_TRUE(first_frame_is_base_aligned);
  EXPECT_NEAR(joint_positions.front().x(), 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions.back().x(), 18.0 * 4.33e-3, 1e-12);
  EXPECT_NEAR(tip_position.x(), 19.0 * 4.33e-3, 1e-12);
  EXPECT_NEAR(tip_position.y(), 0.0, 1e-12);
  EXPECT_NEAR(tip_position.z(), 0.0, 1e-12);
}

TEST(SurgicalToolFk, FirstTiltJointRotatesAboutBaseZTowardPositiveY)
{
  SurgicalTool tool;
  const std::vector<double> pan_angles(NUM_OF_BENDING_JOINTS, 0.0);
  std::vector<double> tilt_angles(NUM_OF_BENDING_JOINTS, 0.0);
  tilt_angles[0] = std::acos(-1.0) / 2.0;

  const auto transforms =
    tool.computeBaseToJointsTransformationMatrices(
    pan_angles, tilt_angles);
  const auto joint_positions = tool.computeJointPositions(transforms);

  const Eigen::Matrix3d expected_orientation =
    Eigen::AngleAxisd(
      std::acos(-1.0) / 2.0,
      Eigen::Vector3d::UnitZ()).toRotationMatrix();

  EXPECT_NEAR(joint_positions[0].x(), 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions[0].y(), 0.0, 1e-12);
  EXPECT_NEAR(joint_positions[1].x(), 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions[1].y(), 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions[1].z(), 0.0, 1e-12);
  const bool first_frame_is_tilted =
    transforms.front().block<3, 3>(0, 0).isApprox(
      expected_orientation, 1e-12);
  EXPECT_TRUE(first_frame_is_tilted);
}

TEST(SurgicalToolFk, SecondPanJointActsTowardPositiveBaseZ)
{
  SurgicalTool tool;
  std::vector<double> pan_angles(NUM_OF_BENDING_JOINTS, 0.0);
  const std::vector<double> tilt_angles(NUM_OF_BENDING_JOINTS, 0.0);
  pan_angles[1] = std::acos(-1.0) / 2.0;

  const auto transforms =
    tool.computeBaseToJointsTransformationMatrices(
    pan_angles, tilt_angles);
  const auto joint_positions = tool.computeJointPositions(transforms);

  // q2 rotates at P2, so its direction is visible at the following boundary.
  EXPECT_NEAR(joint_positions[1].x(), 2.0 * 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions[1].y(), 0.0, 1e-12);
  EXPECT_NEAR(joint_positions[2].x(), 2.0 * 4.33e-3, 1e-12);
  EXPECT_NEAR(joint_positions[2].y(), 0.0, 1e-12);
  EXPECT_NEAR(joint_positions[2].z(), 4.33e-3, 1e-12);
}

TEST(SurgicalToolIk, PositiveTiltProducesSouthNorthDifferential)
{
  SurgicalTool tool;
  const auto wire_length = tool.get_IK_result(0.0, 10.0, 0.0);

  ASSERT_GE(wire_length.size(), 4U);
  EXPECT_NEAR(wire_length[0], wire_length[1], 1e-12);
  EXPECT_LT(wire_length[2], 0.0);
  EXPECT_GT(wire_length[3], 0.0);
}

TEST(SurgicalToolIk, PositivePanProducesEastWestDifferential)
{
  SurgicalTool tool;
  const auto wire_length = tool.get_IK_result(10.0, 0.0, 0.0);

  ASSERT_GE(wire_length.size(), 4U);
  EXPECT_LT(wire_length[0], 0.0);
  EXPECT_GT(wire_length[1], 0.0);
  EXPECT_NEAR(wire_length[2], wire_length[3], 1e-12);
}
