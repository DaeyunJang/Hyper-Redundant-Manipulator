"""Publish ID0/ID1 poses without polling cached TFs or mixing image timestamps."""

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage

from record_pkg.apriltag_pose_geometry import FreshTagPair, TagPose


def pose_message(pose):
    message = PoseStamped()
    message.header.frame_id = pose.parent
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        pose.stamp_ns, 1_000_000_000)
    p, q = message.pose.position, message.pose.orientation
    p.x, p.y, p.z = pose.position
    q.x, q.y, q.z, q.w = pose.orientation
    return message


class AprilTagPoseNode(Node):
    def __init__(self):
        super().__init__('apriltag_pose')
        tag0 = self.declare_parameter('tag0_frame', 'ID0').value
        tag1 = self.declare_parameter('tag1_frame', 'ID1').value
        self.detector_node = self.declare_parameter('detector_node', '/apriltag_node').value
        self.detector_source_id = None
        self.pairs = FreshTagPair(tag0, tag1)
        self.publishers_by_frame = {
            tag0: self.create_publisher(PoseStamped, '/apriltag/tag0/pose', 10),
            tag1: self.create_publisher(PoseStamped, '/apriltag/tag1/pose', 10),
        }
        self.relative_publisher = self.create_publisher(
            PoseStamped, '/apriltag/tag1_in_tag0/pose', 10)
        # BEST_EFFORT subscribes to either TF publisher reliability. These small
        # messages are source-stamped, not regenerated on a wall-clock timer.
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.tf_subscription = self.create_subscription(TFMessage, '/tf', self.on_tf, qos)
        # This Humble rclpy has no per-message MessageInfo callback. Observe the
        # configured detector's sole TF endpoint in the graph instead.
        self.source_timer = self.create_timer(0.5, self.refresh_detector_source)
        self.get_logger().info(
            f'AprilTag recording poses: {tag0}, {tag1}; relative pose is '
            f'{tag1} expressed in {tag0}, only for identical image timestamps.')

    def refresh_detector_source(self):
        endpoints = self.get_publishers_info_by_topic('/tf')
        gids = {bytes(endpoint.endpoint_gid) for endpoint in endpoints
                if (endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name)
                == self.detector_node}
        if len(gids) > 1:
            self.get_logger().warning(
                f'Multiple TF publishers named {self.detector_node}; cannot identify tag source.',
                throttle_duration_sec=5.0)
            self.detector_source_id = None
        elif gids:
            self.detector_source_id = next(iter(gids))

    def on_tf(self, message):
        if self.detector_source_id is None:
            self.refresh_detector_source()
        if self.detector_source_id is None:
            return
        for transform in message.transforms:
            if transform.child_frame_id not in self.publishers_by_frame:
                continue
            p, q = transform.transform.translation, transform.transform.rotation
            stamp = transform.header.stamp
            pose = TagPose(transform.header.frame_id, transform.child_frame_id,
                           stamp.sec * 1_000_000_000 + stamp.nanosec,
                           (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            try:
                previous_changes = self.pairs.source_changes
                camera_pose, relative = self.pairs.ingest(pose, self.detector_source_id)
                if self.pairs.source_changes != previous_changes:
                    self.get_logger().info(
                        'AprilTag TF publisher changed; starting a new source timestamp epoch.')
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)
                continue
            if camera_pose is not None:
                self.publishers_by_frame[camera_pose.child].publish(pose_message(camera_pose))
            if relative is not None:
                self.relative_publisher.publish(pose_message(relative))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagPoseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
