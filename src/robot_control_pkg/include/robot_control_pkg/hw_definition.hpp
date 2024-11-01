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
#define NUM_OF_MOTORS       2
#define GEAR_RATIO          51
#define GEAR_RATIO_44       44
#define GEAR_RATIO_3_9      3.9
#define ENCODER_CHANNEL     4
#define ENCODER_RESOLUTION  1024
#define INC_PER_ROT_44      GEAR_RATIO_44 * ENCODER_CHANNEL * ENCODER_RESOLUTION
#define INC_PER_ROT_3_9     GEAR_RATIO_3_9 * ENCODER_CHANNEL * ENCODER_RESOLUTION
#define DIRECTION_COUPLER   1      // if not, use 1

/** Motor control mode **/
/**
 * @brief It means that the ETA(Estimation Time Arrive) is same for all motors (arrive at same time)
 *        If not use it, just each motors move same velocity (not arrive at same time)
*/
#define MOTOR_CONTROL_SAME_DURATION 1
#define PERCENT_100 50

//===============================
// Continuum Manipulator Paremeters
//===============================
#define DOF               1     // Degree of freedom

#define NUM_OF_JOINT      8     // ea
#define SEGMENT_ARC       5.0  // mm
#define SEGMENT_DIAMETER  9     // mm
#define WIRE_DISTANCE     3.63   // mm
// #define NUM_OF_JOINT      8     // ea
// #define SEGMENT_ARC       6.92  // mm
// #define SEGMENT_DIAMETER  3     // mm
// #define WIRE_DISTANCE     1.2   // mm

#define SHIFT             5.41  // degree
#define SHIFT_THRESHOLD   5.0   // deg

#define MAX_BENDING_DEGREE   90.0 // degree
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