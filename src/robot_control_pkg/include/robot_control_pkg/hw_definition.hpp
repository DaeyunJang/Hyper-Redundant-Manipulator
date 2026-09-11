#pragma once
/**
 * @file hw_definition.hpp
 * @author daeyun (bigyun9375@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef HW_DEFINITION_HPP_
#define HW_DEFINITION_HPP_
//===============================
// Motor Parameters
//===============================
#define OP_MODE             0x08  // CSP:0x08 CSV:0x09
#define NUM_OF_MOTORS       4
#define GEAR_RATIO          51
#define GEAR_RATIO_44       44
#define GEAR_RATIO_3_9      3.9
#define ENCODER_CHANNEL     4
#define ENCODER_RESOLUTION  1024
#define INC_PER_ROT_44      GEAR_RATIO_44 * ENCODER_CHANNEL * ENCODER_RESOLUTION
#define INC_PER_ROT_3_9     GEAR_RATIO_3_9 * ENCODER_CHANNEL * ENCODER_RESOLUTION
#define DIRECTION_COUPLER   1      // if not, use 1

#define MOTOR_SOFTWARE_LIMIT 700000
#define TENSION_LIMIT 2000  // g

/** Motor control mode **/
/**
 * @brief It means that the ETA(Estimation Time Arrive) is same for all motors (arrive at same time)
 *        If not use it, just each motors move same velocity (not arrive at same time)
*/
#define MOTOR_CONTROL_SAME_DURATION 0
#define PERCENT_100 100

//===============================
// Continuum Manipulator Paremeters
//===============================
#define DOF               2     // Pan and tilt bending degrees of freedom

// One fixed proximal segment plus eighteen alternating one-axis revolute
// joints gives nineteen geometric segments: os, 17*l, le.
#define NUM_OF_SEGMENTS         19
#define NUM_OF_JOINT_PAIRS       9
#define NUM_OF_BENDING_JOINTS   18
#define NUM_OF_PAN_JOINTS        9
#define NUM_OF_TILT_JOINTS       9

static_assert(
  NUM_OF_SEGMENTS == NUM_OF_BENDING_JOINTS + 1,
  "The fixed proximal segment requires one more segment than bending joints.");
static_assert(
  NUM_OF_BENDING_JOINTS == 2 * NUM_OF_JOINT_PAIRS,
  "Bending joints must form complete pan/tilt pairs.");
static_assert(
  NUM_OF_PAN_JOINTS == NUM_OF_JOINT_PAIRS &&
  NUM_OF_TILT_JOINTS == NUM_OF_JOINT_PAIRS,
  "Pan and tilt joint counts must match the joint-pair count.");

// Modified-DH centerline distances (mm): Base-to-q1, qi-to-q(i+1), q18-to-tip.
#define PROXIMAL_OFFSET_LENGTH   4.33
#define BENDING_JOINT_SPACING    4.33
#define DISTAL_OFFSET_LENGTH     4.33
#define SEGMENT_ARC       5.5  // mm
#define SEGMENT_DIAMETER  9     // mm
#define WIRE_DISTANCE     3.63   // mm
#define TOTAL_LENGTH      PROXIMAL_OFFSET_LENGTH + BENDING_JOINT_SPACING*(NUM_OF_BENDING_JOINTS-1) + DISTAL_OFFSET_LENGTH
#define SEGMENT_ARC_CENTER_TO_SEGMENT_CENTER  10.5  // mm

#define SHIFT             5.41  // degree
#define SHIFT_THRESHOLD   5.0   // deg

#define MAX_BENDING_DEGREE   60.0 // degree
#define MAX_FORCEPS_RAGNE_DEGREE 30.0  // degree
#define MAX_FORCEPS_RAGNE_MM 2.0  // mm (nor int)

// #define JOINT_INTERVAL    3   // mm

//===============================
// Loadcell Parameters
//===============================
#define LOADCELL_THRESHOLD  1000.0

//===============================
// Motor Specification
// DCX22 series
// MAXON.co
//===============================
typedef struct {
  float gear_ratio = 44;
  int encoder_channel = 3;
  int encoder_resolution = 1024;
  int inc_per_rot = gear_ratio * encoder_channel * encoder_resolution;
} DCX22_G44;

typedef struct {
  float gear_ratio = 3.9;
  int encoder_channel = 3;
  int encoder_resolution = 1024;
  int inc_per_rot = gear_ratio * encoder_channel * encoder_resolution;
} DCX22_G3_9;

#endif
