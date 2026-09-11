#pragma once
/**
 * @file surgical_tool.hpp
 * @author daeyun (bigyun9375@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef SURGICAL_TOOL_HPP_
#define SURGICAL_TOOL_HPP_

#include <iostream>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <vector>
#include <math.h>
#include <tuple>
#include <Eigen/Dense>

#include "hw_definition.hpp"

/**
 * @authors DY, JYoo, JKim
 * @brief Structure of Surgical tool (Forceps)
 * @unit Degree
 * @param pAngle : Pan angle (East(-) & West(+))
 * @param tAngle : Tilt angle (South(+) & North(-))
 * @param num_joint_pairs number of alternating pan/tilt joint pairs
 * @param arc       degree of arc of the segment part (mm)
 * @param diameter  diameter of the segment (mm)
 * @param disWire   distance between the center and the center of ellipse (mm)
 * @param shift     center of wire in tilt direction shifted during pan motion
 */
struct structure {
  int num_joint_pairs;
  float pAngle;
  float tAngle;
  float arc;
  float diameter;
  float disWire;
  float shift;
  float arc_center_to_seg_center;
};

class SurgicalTool
{
public:
  /**
   * @brief Construct a new Surgical Tool object
   */
  SurgicalTool();

  /**
   * @brief Destroy the Surgical Tool object
   */
  ~SurgicalTool();

  /**
   * @brief make 1 object of the surgical tool
   *
   */
  struct structure surgicaltool_;

  /**
   * @authors DY
   * @brief initialize of surgical tool
   */
  void init_surgical_tool(
    int num_joint_pairs,
    float arc,
    float diameter,
    float disWire,
    float shift,
    float arc_center_to_seg_center
  );

  // **************************
  // Limitation of workspace
  // **************************
  double max_bending_deg_ = MAX_BENDING_DEGREE;
  double max_forceps_deg_ = MAX_FORCEPS_RAGNE_DEGREE;

  /**
   * @brief target angle of the manipulator
   * @unit degree
   */
  double pAngle_ = 0;   // East * West
	double tAngle_ = 0;   // South * North
  /**
   * @brief target length for moving wire using motor
   * @unit mm
   */
	double wrLengthWest_, wrLengthEast_, wrLengthSouth_, wrLengthNorth_;
  double wrLengthGrip;

  /**
   * @brief Set the bending angle object
   * @unit degree
   * @note consider DOF
   * @param pAngle
   * @param tAngle
   */
  void set_bending_angle(double pAngle);
  void set_bending_angle(double pAngle, double tAngle);

  /**
   * @brief Set the forceps angle object
   * @unit degree
   * @param angle
   */
  void set_forceps_angle(double angle);

  /**
   * @brief Get the bending kinematic result object
   * @param pAngle  pan angle
   * @param tAngle  tilt angle
   * @param grip    grip angle
   * @note   Also, class has the value : this->wrLength** variables has the target values
   *         or use the return values
   * @return std::vector<double ...>
   */
  std::vector<double> get_IK_result(double pAngle, double tAngle, double grip);

  /**
   * @brief calculate the inverse-kinematics
   */
  void inverse_kinematics();

  /**
   * @brief Calculate one modified-DH transform from frame i-1 to frame i.
   * @param joint_angle q_i in radians
   * @param twist_angle alpha_(i-1) in radians
   * @param previous_link_length r_(i-1) in metres
   * @return Eigen::Matrix4d
   */
  Eigen::Matrix4d computeTransformationMatrix(
    const double& joint_angle,
    const double& twist_angle,
    const double& previous_link_length) const;

  /**
   * @brief Calculate Base-to-joint transforms for all 18 bending joints.
   * @param pan_angles 18 entries; active at zero-based even indices
   * @param tilt_angles 18 entries; active at zero-based odd indices
   * @return std::vector<Eigen::Matrix4d>
   */
  std::vector<Eigen::Matrix4d> computeBaseToJointsTransformationMatrices(
    const std::vector<double>& pan_angles,
    const std::vector<double>& tilt_angles) const;

  /**
   * @brief Get XYZ translation from a homogeneous transform.
   *
   * @param T Homogeneous Transfrom Matrix
   * @return Eigen::Vector3d
   * @example
   *  // e.g. Total 60 deg with 6 joints(10 deg each)
   *  std::vector<double> pan_angles(NUM_OF_BENDING_JOINTS, 0.0);
   *  std::vector<double> tilt_angles(NUM_OF_BENDING_JOINTS, 0.0);
   *  auto tf_matrices = computeBaseToJointsTransformationMatrices(
   *    pan_angles, tilt_angles);
   *  auto joints_xyz = computeJointPositions(tf_matrices);
   *  std::cout << joints_xyz << std::endl;
   */
  Eigen::Vector3d extractXYZfromTransformMatrix(const Eigen::Matrix4d& T) const;

  /**
   * @brief Get the 18 DH joint positions in Base coordinates.
   * @param transforms Base-to-joint homogeneous transforms
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> computeJointPositions(
    const std::vector<Eigen::Matrix4d>& transforms) const;

  /**
   * @brief Get the tool-tip position after the fixed distal offset.
   * @param transforms Base-to-joint homogeneous transforms
   * @return Tool-tip position in Base coordinates
   */
  Eigen::Vector3d computeEndEffectorPosition(
    const std::vector<Eigen::Matrix4d>& transforms) const;

  /**
   * @brief Get the Base-to-tip homogeneous transform.
   * @param transforms Base-to-joint homogeneous transforms
   * @return Base-to-tip transform
   */
  Eigen::Matrix4d computeEndEffectorTransformation(
    const std::vector<Eigen::Matrix4d>& transforms) const;

  /**
   * @brief make input variable to 'mm' unit
   * @return * float
   */
  float tomm();

  /**
   * @brief make input variable to 'rad' unit
   * @return float
   */
  float torad();

  /**
   * @brief make input variable to 'rad' unit
   * @return float
   */
  float todeg();

private:
  // const double PI = acos(-1);
  const double PI_ = M_PI;

  double deg_ = M_PI / 180;
	double rad_ = 180 / M_PI;
	double mm_ = 0.001;

  double target_forceps_angle_ = 30;

  double alpha_;

  float release_gain_ = 1.0;
};

#endif
