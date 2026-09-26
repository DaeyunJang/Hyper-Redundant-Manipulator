#pragma once

#include "hw_definition.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// Read-only record of THIS executable's constants, not a drive/firmware readback.
// Ignore parameter overrides: a YAML/CLI setting must not falsify build metadata.
inline void declare_hardware_metadata(rclcpp::Node & node)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Compiled hardware definition; not physical driver readback.";
  const auto integer = [&](const char * name, int64_t value) {
    node.declare_parameter<int64_t>(std::string("hardware.") + name, value, descriptor, true);
  };
  const auto number = [&](const char * name, double value) {
    node.declare_parameter<double>(std::string("hardware.") + name, value, descriptor, true);
  };
  const auto label = [&](const char * name, const std::string & value) {
    node.declare_parameter<std::string>(std::string("hardware.") + name, value, descriptor, true);
  };
  label("source", "compiled hw_definition.hpp");
  integer("schema_version", 1);
  integer("OP_MODE", OP_MODE);
  label("OP_MODE_label", OP_MODE == 0x08 ? "CSP" : (OP_MODE == 0x09 ? "CSV" : "unknown"));
  integer("NUM_OF_MOTORS", NUM_OF_MOTORS);
  integer("ENCODER_CHANNEL", ENCODER_CHANNEL);
  integer("ENCODER_RESOLUTION", ENCODER_RESOLUTION);
  number("GEAR_RATIO", GEAR_RATIO);
  number("GEAR_RATIO_44", GEAR_RATIO_44);
  number("GEAR_RATIO_3_9", GEAR_RATIO_3_9);
  number("INC_PER_ROT_44", (INC_PER_ROT_44));
  number("INC_PER_ROT_3_9", (INC_PER_ROT_3_9));
  integer("DIRECTION_COUPLER", DIRECTION_COUPLER);
  integer("MOTOR_SOFTWARE_LIMIT", MOTOR_SOFTWARE_LIMIT);
  number("TENSION_LIMIT", TENSION_LIMIT);
  integer("MOTOR_CONTROL_SAME_DURATION", MOTOR_CONTROL_SAME_DURATION);
  integer("PERCENT_100", PERCENT_100);
  integer("DOF", DOF);
  integer("NUM_OF_SEGMENTS", NUM_OF_SEGMENTS);
  integer("NUM_OF_JOINT_PAIRS", NUM_OF_JOINT_PAIRS);
  integer("NUM_OF_BENDING_JOINTS", NUM_OF_BENDING_JOINTS);
  integer("NUM_OF_PAN_JOINTS", NUM_OF_PAN_JOINTS);
  integer("NUM_OF_TILT_JOINTS", NUM_OF_TILT_JOINTS);
  number("PROXIMAL_OFFSET_LENGTH", PROXIMAL_OFFSET_LENGTH);
  number("BENDING_JOINT_SPACING", BENDING_JOINT_SPACING);
  number("DISTAL_OFFSET_LENGTH", DISTAL_OFFSET_LENGTH);
  number("SEGMENT_ARC", SEGMENT_ARC);
  number("SEGMENT_DIAMETER", SEGMENT_DIAMETER);
  number("WIRE_DISTANCE", WIRE_DISTANCE);
  number("TOTAL_LENGTH", (TOTAL_LENGTH));
  number("SEGMENT_ARC_CENTER_TO_SEGMENT_CENTER", SEGMENT_ARC_CENTER_TO_SEGMENT_CENTER);
  number("SHIFT", SHIFT);
  number("SHIFT_THRESHOLD", SHIFT_THRESHOLD);
  number("MAX_BENDING_DEGREE", MAX_BENDING_DEGREE);
  number("MAX_FORCEPS_RAGNE_DEGREE", MAX_FORCEPS_RAGNE_DEGREE);
  number("MAX_FORCEPS_RAGNE_MM", MAX_FORCEPS_RAGNE_MM);
  number("LOADCELL_THRESHOLD", LOADCELL_THRESHOLD);
  label("length_unit", "mm");
  label("angle_unit", "degree");
  label("tension_unit", "g");
  label("motor_position_unit", "encoder counts");
}
