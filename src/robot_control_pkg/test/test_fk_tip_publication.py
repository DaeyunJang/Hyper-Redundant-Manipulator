"""Source-contract checks for the recorded FK tip; no ROS nodes run."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src/control_node.cpp').read_text()
HEADER = (PACKAGE / 'include/robot_control_pkg/control_node.hpp').read_text()


def _block_after(text, anchor):
    """Extract the braced block following an exact C++ source anchor."""
    start = text.index('{', text.index(anchor))
    depth = 1
    cursor = start + 1
    while depth:
        if text[cursor] == '{':
            depth += 1
        elif text[cursor] == '}':
            depth -= 1
        cursor += 1
    return text[start + 1:cursor - 1]


def test_stamped_publisher_is_additive_and_reliable():
    assert '#include "geometry_msgs/msg/point_stamped.hpp"' in HEADER
    assert 'rclcpp::Publisher<geometry_msgs::msg::PointStamped>' in HEADER
    assert '"kinematics/fk_tip_position", qos_reliable_latest' in SOURCE
    assert '"tool_endeffector_pose", qos_reliable_latest' in SOURCE


def test_tip_is_published_only_with_a_new_angle_sample():
    worker = _block_after(
        SOURCE,
        'void ControlNode::run_position_with_admittance_control_thread()')
    fresh_frame = _block_after(worker, 'if (new_frame)')
    publish = 'fk_tip_position_publisher_->publish(fk_tip_position);'
    assert SOURCE.count(publish) == 1
    assert publish in fresh_frame
    old_publish = (
        'tool_endeffector_pose_publisher_->publish(tool_endeffector_pose_);')
    assert old_publish in fresh_frame
    finite_guard = fresh_frame.index('if (!tip.allFinite())')
    assert finite_guard < fresh_frame.index(publish)


def test_stamped_tip_preserves_source_header_and_same_metric_fk():
    worker = _block_after(
        SOURCE,
        'void ControlNode::run_position_with_admittance_control_thread()')
    fresh_frame = _block_after(worker, 'if (new_frame)')
    assert 'sample.pan_relative, sample.tilt_relative' in fresh_frame
    assert 'computeEndEffectorPosition(transforms)' in fresh_frame
    assert 'fk_tip_position.header = sample.header;' in fresh_frame
    assert 'now()' not in fresh_frame
    for axis in 'xyz':
        assert f'fk_tip_position.point.{axis} = tip.{axis}();' in fresh_frame
    # Existing FK unit tests check straight-chain SI units (19 * 4.33e-3 m).


def test_angle_input_rejects_invalid_frame_and_repeated_source_stamps():
    assert 'msg->header.frame_id != "hrm_base"' in SOURCE
    assert 'stamp_ns(msg->header.stamp) <= 0' in SOURCE
    increasing_stamp_guard = (
        'stamp_ns(msg->header.stamp) <= stamp_ns(segment_angle_.header.stamp)')
    assert increasing_stamp_guard in SOURCE
