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
 * @param num_joint number of joint of continuum parts
 * @param arc       degree of arc of the segment part (mm)
 * @param diameter  diameter of the segment (mm)
 * @param disWire   distance between the center and the center of ellipse (mm)
 * @param shift     center of wire in tilt direction shifted during pan motion
 */
struct structure {
  int num_joint;
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
    int num_joint,
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
   * @brief calculate the Homogeneous transform matrix of each joint
   * @param theta radian
   * @return Eigen::Matrix4d
   */
  Eigen::Matrix4d computeTransformationMatrix(const double& theta);

  /**
   * @brief calculate the transformation matrix of each joint
   * @return std::vector<Eigen::Matrix4d> 
   */
  std::vector<Eigen::Matrix4d> computeBaseToJointsTransformationMatrices(const std::vector<double>& joint_angles);
  
  /**
   * @brief get <x,y> from given transform matrix
   * 
   * @param T Homogeneous Transfrom Matrix
   * @return Eigen::Vector2d 
   * @example
   *  // e.g. Total 60 deg with 6 joints(10 deg each)
   *  std::vector<double> joint_angles = [0.17, 0.17, 0.17, 0.17, 0.17, 0.17];
   *  auto tf_matrices = computeBaseToJointsTransformationMatrices(joint_angles);
   *  auto joints_xy = computeJointPositions(tf_matrices);
   *  std::cout << joints_xy << std::endl;
   */
  Eigen::Vector2d extractXYfromTransformMatrix(const Eigen::Matrix4d& T);

  /**
   * @brief Get the Joint Positions object
   * @param frame Group of the homogeneous transform matrix
   * @return std::vector<Eigen::Vector2d> 
   */
  std::vector<Eigen::Vector2d> computeJointPositions(const std::vector<Eigen::Matrix4d>& transforms);

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