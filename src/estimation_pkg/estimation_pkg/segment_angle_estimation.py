import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy

from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray
from custom_interfaces.msg import SegmentAngle
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits
from tf2_ros import TransformBroadcaster

from estimation_pkg.postprocess import RBSC
from estimation_pkg.runtime import configure_image_transport
from estimation_pkg.crop_depth import crop_depth_calibration
# from postprocess import RBSC

import threading
import os
import cv2
import json
import numpy as np
import time
import signal

print(f"[segment] Current Working Directory : {os.getcwd()}")


def validate_roi(x, y, width, height):
    """ROI is a top-left pixel origin plus positive pixel width/height."""
    values = (x, y, width, height)
    for name, value, minimum in zip(
            ('roi_x', 'roi_y', 'roi_width', 'roi_height'), values, (0, 0, 1, 1)):
        if type(value) is not int or value < minimum:
            raise ValueError(f'{name} must be an integer >= {minimum}.')
    return values


class SegmentEstimationNode(Node):
    def __init__(self, roi_config_file='config_ROI_ref.json'):
        super().__init__('segment_estimation_node')
        self.roi_config = self.load_config(roi_config_file)
        defaults = validate_roi(*(self.roi_config[key] for key in ('x', 'y', 'w', 'h')))
        names = ('roi_x', 'roi_y', 'roi_width', 'roi_height')
        for name, value in zip(names, defaults):
            self.declare_parameter(name, value, ParameterDescriptor(
                read_only=True,
                description='Fixed estimation crop in pixels; restart estimation to change ROI.'))
        self.roi_x, self.roi_y, self.roi_w, self.roi_h = validate_roi(
            *(self.get_parameter(name).value for name in names))
        self.get_logger().info(
            f'Fixed estimation ROI: x={self.roi_x}, y={self.roi_y}, '
            f'width={self.roi_w}, height={self.roi_h}. '
            'Full camera images and AprilTag input are unchanged.')
        self.current_frame = None
        self.current_frame_ROI = None
        self.current_frame_encoding = None
        self.current_depth = None
        self.color_stamp = None
        self.depth_stamp = None
        self.depth_scale = None
        self.camera_intrinsics = None
        self.depth_archive_camera_info = None
        self.camera_frame_id = None
        self.received_first_color = False
        self.received_first_depth = False
        self.received_first_camera_info = False
        self.published_first_base_tf = False
        self.published_first_segment_center_tf = False
        self.published_first_segment_angle = False
        self.previous_angle_arrays = None
        self.previous_angle_stamp_sec = None
        self.processed_frame_count = 0
        self.failed_frame_count = 0
        self.overwritten_frame_count = 0
        self.last_processing_ms = 0.0
        self.last_result_age_ms = 0.0
        self.visualized_frame_count = 0
        self.color_received_count = 0
        self.depth_received_count = 0
        self.rate_window_start = time.monotonic()
        self.rate_window_color_count = 0
        self.rate_window_depth_count = 0
        self.rate_window_processed_count = 0
        self.frame_lock = threading.Lock()
        self.result_lock = threading.Lock()
        self.latest_visualization = None
        self.latest_crop = None
        self.latest_crop_depth = None
        self.previous_marker_keys = None
        self.rbsc = RBSC()
        performance_config = self.rbsc.config.get('performance', {})
        self.opencv_num_threads = max(
            1, int(performance_config.get('opencv_num_threads', 1))
        )
        cv2.setNumThreads(self.opencv_num_threads)
        # Tiny quartic/3x3 solves do not benefit from a 24-thread BLAS pool.
        # Apply this for ros2 run as well as launch and retain the controller.
        self.blas_thread_limit = threadpool_limits(
            limits=max(1, int(performance_config.get('blas_num_threads', 1))),
            user_api='blas',
        )
        self.max_depth_age_sec = max(
            0.0, float(performance_config.get('max_depth_age_sec', 0.1))
        )
        self.debug_timing_enabled = bool(
            performance_config.get('debug_timing_enabled', False)
        )
        self.debug_rate_enabled = bool(
            performance_config.get('debug_rate_enabled', False)
        )
        self.timing_log_interval_frames = max(
            0, int(performance_config.get('timing_log_interval_frames', 60))
        )
        self.rate_log_interval_sec = max(
            0.0, float(performance_config.get('rate_log_interval_sec', 5.0))
        )
        self.visualization_curve_max_points = max(
            2, int(performance_config.get(
                'visualization_curve_max_points', 251
            ))
        )

        # self.joint_angle = np.array(0)
        print("============================")
        print(os.getcwd())
        print(os.path.abspath(__file__))
        print("============================")

        self.declare_parameter('debug_timing_enabled', self.debug_timing_enabled)
        self.declare_parameter('debug_rate_enabled', self.debug_rate_enabled)
        self.debug_timing_enabled = self.get_parameter('debug_timing_enabled').value
        self.debug_rate_enabled = self.get_parameter('debug_rate_enabled').value
        if self.debug_rate_enabled and self.rate_log_interval_sec > 0.0:
            self.create_timer(self.rate_log_interval_sec, self.report_rates)
        self.base_tf_broadcaster = TransformBroadcaster(self)
        self.declare_parameter('qos_depth', 1)
        self.declare_parameter(
        'color_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter(
        'aligned_depth_topic',
        '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
        'camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_scale', 0.0)
        qos_depth = self.get_parameter('qos_depth').value
        color_topic = self.get_parameter('color_topic').value
        aligned_depth_topic = self.get_parameter('aligned_depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_scale_override = self.get_parameter('depth_scale').value
        QOS_SENSOR = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=max(1, qos_depth),
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        QOS_CONTROL_LATEST = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        QOS_VISUALIZATION_LATEST = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
    )

        # self.realsense_subscriber = RealSenseSubscriber()
        # color rectified image. RGB format
        self.br_rgb = CvBridge()  # CvBridge can deal with several process --> can use both subscription and publishment

        self.current_frame_flag = False
        self.color_image_rect_raw_subscriber = self.create_subscription(
        Image,
        color_topic,
        self.color_image_rect_raw_callback,
        QOS_SENSOR)
        self.aligned_depth_subscriber = self.create_subscription(
        Image,
        aligned_depth_topic,
        self.aligned_depth_callback,
        QOS_SENSOR)
        self.camera_info_subscriber = self.create_subscription(
        CameraInfo,
        camera_info_topic,
        self.camera_info_callback,
        QOS_SENSOR)
        self.get_logger().info(
        'Color, aligned-depth, and color-camera-info subscribers are created.')

        self.segment_crop_image_publisher = self.create_publisher(
      Image,
      'estimated_segment_crop_image',
      QOS_VISUALIZATION_LATEST
    )
        self.crop_depth_publisher = self.create_publisher(
            Image, 'estimated_segment_crop_depth/image_raw', QOS_VISUALIZATION_LATEST)
        self.crop_depth_info_publisher = self.create_publisher(
            CameraInfo, 'estimated_segment_crop_depth/camera_info', QOS_VISUALIZATION_LATEST)
        self.crop_depth_metadata_publisher = self.create_publisher(
            String, 'estimated_segment_crop_depth/metadata', QOS_VISUALIZATION_LATEST)
        self.segment_body_binary_image_publisher = self.create_publisher(
      Image,
      'estimated_segment_body_binary_image',
      QOS_VISUALIZATION_LATEST
    )
        self.segment_skeleton_image_publisher = self.create_publisher(
      Image,
      'estimated_segment_skeleton_image',
      QOS_VISUALIZATION_LATEST
    )
        self.centerline_points_publisher = self.create_publisher(
      PointCloud2,
      'estimated_segment_centerline_points',
      QOS_VISUALIZATION_LATEST
    )
        self.reconstruction_markers_publisher = self.create_publisher(
      MarkerArray,
      'estimated_segment_reconstruction_markers',
      QOS_VISUALIZATION_LATEST
    )
        self.segment_angle_publisher = self.create_publisher(
      SegmentAngle,
      'estimated_segment_angle',
      QOS_CONTROL_LATEST
    )
        self.tip_position_publisher = self.create_publisher(
            PointStamped, 'estimated_tip_position', QOS_CONTROL_LATEST)
        self.tip_position_unscaled_publisher = self.create_publisher(
            PointStamped, 'estimated_tip_position_unscaled', QOS_CONTROL_LATEST)

        self.event = threading.Event()
        self.frame_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.segment_estimation_thread = threading.Thread(
        target=self.process, daemon=True)
        self.realtime_show_thread = threading.Thread(
        target=self.realtime_show, daemon=True)

        self.segment_estimation_thread.start()
        self.realtime_show_thread.start()

    def load_config(self, config_file):

        package_share_directory = get_package_share_directory('estimation_pkg')
        config_path = os.path.join(package_share_directory, config_file)
        print(f'[segment_angle_estimation.py] config.json PATH: {config_path}', flush=True)

        print(f'Load {config_file}', flush=True)
        print(f'===== json list ======', flush=True)
        with open(config_path, 'r') as f:
            config = json.load(f)
            # print
            for key, value in config.items():
                print(f'{key} : {value}', flush=True)
        print(f'===== json list end ===', flush=True)

        return config

    def crop_to_roi(self, frame, stream_name):
        """Return a view only when the entire configured rectangle is present."""
        height, width = frame.shape[:2]
        if self.roi_x + self.roi_w > width or self.roi_y + self.roi_h > height:
            self.get_logger().warning(
                f'Rejecting {stream_name} frame {width}x{height}: ROI '
                f'(x={self.roi_x}, y={self.roi_y}, width={self.roi_w}, '
                f'height={self.roi_h}) exceeds image bounds. '
                'Stop estimation, choose a valid ROI, then start it again.',
                throttle_duration_sec=2.0)
            return None
        return frame[self.roi_y:self.roi_y + self.roi_h,
                     self.roi_x:self.roi_x + self.roi_w]

    def color_image_rect_raw_callback(self, data):
        self.color_received_count += 1
        if data.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().error(
                f'Unsupported color image encoding: {data.encoding}'
            )
            return
        frame = self.br_rgb.imgmsg_to_cv2(data, 'passthrough')
        color_roi = self.crop_to_roi(frame, 'color')
        if color_roi is None:
            self.failed_frame_count += 1
            with self.frame_lock:
                self.current_frame_flag = False
                self.current_frame_ROI = None
                self.color_stamp = None
            with self.result_lock:
                self.latest_crop = None
            return
        if self.segment_crop_image_publisher.get_subscription_count() > 0:
            # Keep DDS image callbacks short. Serialization/publication belongs
            # to the visualization worker, even if depth or the fit is missing.
            with self.result_lock:
                self.latest_crop = (color_roi, data.header, data.encoding)
                self.event.set()

        if not self.received_first_color:
            self.get_logger().info(
          f'First color frame received; encoding={data.encoding}, '
          f'ROI shape={color_roi.shape}.')
            self.received_first_color = True

        with self.frame_lock:
            if self.current_frame_flag:
                self.overwritten_frame_count += 1
            self.current_frame_flag = True
            self.color_stamp = data.header.stamp
            self.current_frame_ROI = color_roi
            self.current_frame_encoding = data.encoding
        self.frame_event.set()

    def aligned_depth_callback(self, data):
        self.depth_received_count += 1
        depth = self.br_rgb.imgmsg_to_cv2(data, 'passthrough')
        depth_roi = self.crop_to_roi(depth, 'aligned-depth')
        if depth_roi is None:
            self.failed_frame_count += 1
            with self.frame_lock:
                self.current_depth = None
                self.depth_stamp = None
                self.depth_scale = None
            return
        if not self.received_first_depth:
            self.get_logger().info(
          f'First aligned-depth frame received; encoding={data.encoding}, '
          f'shape={depth.shape}.')
            self.received_first_depth = True
        if self.depth_scale_override > 0.0:
            depth_scale = float(self.depth_scale_override)
        elif data.encoding in ('16UC1', 'mono16'):
            depth_scale = 0.001
        elif data.encoding == '32FC1':
            depth_scale = 1.0
        else:
            self.get_logger().warning(
          f'Unknown depth encoding {data.encoding}; assuming meters.')
            depth_scale = 1.0

        with self.frame_lock:
            self.current_depth = depth_roi
            self.depth_stamp = data.header.stamp
            self.depth_scale = depth_scale
            archive_info = getattr(self, 'depth_archive_camera_info', None)
        archive_publisher = getattr(self, 'crop_depth_publisher', None)
        if archive_publisher is not None and archive_publisher.get_subscription_count() > 0:
            # Queue only a view/reference in the DDS callback. Calibration,
            # serialization and publication run in the visualization worker.
            with self.result_lock:
                self.latest_crop_depth = (
                    depth_roi, data.header, data.encoding, depth_scale,
                    depth.shape, archive_info)
                self.event.set()

    def camera_info_callback(self, data):
        intrinsics = {
        'fx': float(data.k[0]),
        'fy': float(data.k[4]),
        'cx': float(data.k[2]),
        'cy': float(data.k[5]),
    }
        if intrinsics['fx'] <= 0.0 or intrinsics['fy'] <= 0.0:
            self.get_logger().warning('Received invalid camera intrinsics.')
            return
        if not self.received_first_camera_info:
            self.get_logger().info(
          f'First CameraInfo received; frame_id={data.header.frame_id}.')
            self.received_first_camera_info = True
        with self.frame_lock:
            self.camera_intrinsics = intrinsics
            self.camera_frame_id = data.header.frame_id
            self.depth_archive_camera_info = data

    def publish_base_transform(
            self,
            stamp,
            camera_frame_id,
            base_frame_id,
            base_origin_camera_xyz,
            quaternion_camera_from_base_xyzw):
        """Publish ^C T_B with Camera as parent and HRM Base as child."""
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = camera_frame_id
        transform.child_frame_id = base_frame_id

        transform.transform.translation.x = float(base_origin_camera_xyz[0])
        transform.transform.translation.y = float(base_origin_camera_xyz[1])
        transform.transform.translation.z = float(base_origin_camera_xyz[2])

        transform.transform.rotation.x = float(
            quaternion_camera_from_base_xyzw[0]
        )
        transform.transform.rotation.y = float(
            quaternion_camera_from_base_xyzw[1]
        )
        transform.transform.rotation.z = float(
            quaternion_camera_from_base_xyzw[2]
        )
        transform.transform.rotation.w = float(
            quaternion_camera_from_base_xyzw[3]
        )
        self.base_tf_broadcaster.sendTransform(transform)

        if not self.published_first_base_tf:
            self.get_logger().info(
                f'Publishing TF {camera_frame_id} -> {base_frame_id}; '
                f'base origin in camera=({base_origin_camera_xyz[0]:.4f}, '
                f'{base_origin_camera_xyz[1]:.4f}, '
                f'{base_origin_camera_xyz[2]:.4f}) m.'
            )
            self.published_first_base_tf = True

    def publish_estimated_segment_transforms(
            self,
            stamp,
            base_frame_id,
            center_points_xyz,
            frame_rotations):
        """Publish measured centers with filtered D-H frame orientations."""
        center_points_xyz = np.asarray(center_points_xyz, dtype=float)
        frame_rotations = np.asarray(frame_rotations, dtype=float)
        if (center_points_xyz.ndim != 2 or
                center_points_xyz.shape[1] != 3 or
                frame_rotations.shape != (
                    center_points_xyz.shape[0], 3, 3)):
            raise ValueError(
                'Estimated segment centers and rotations have invalid shapes.'
            )

        transforms = []
        quaternions = Rotation.from_matrix(frame_rotations).as_quat()
        for index, (center_xyz, quaternion_xyzw) in enumerate(zip(
                center_points_xyz, quaternions)):

            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = base_frame_id
            transform.child_frame_id = (
                f'estimated_segment_center_{index + 1:02d}'
            )
            transform.transform.translation.x = float(center_xyz[0])
            transform.transform.translation.y = float(center_xyz[1])
            transform.transform.translation.z = float(center_xyz[2])
            transform.transform.rotation.x = float(quaternion_xyzw[0])
            transform.transform.rotation.y = float(quaternion_xyzw[1])
            transform.transform.rotation.z = float(quaternion_xyzw[2])
            transform.transform.rotation.w = float(quaternion_xyzw[3])
            transforms.append(transform)

        self.base_tf_broadcaster.sendTransform(transforms)
        if not self.published_first_segment_center_tf:
            self.get_logger().info(
                f'Publishing {len(transforms)} measured segment-center TFs '
                f'under {base_frame_id}.'
            )
            self.published_first_segment_center_tf = True

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def wrapped_angular_velocity(current, previous, dt):
        angle_difference = np.arctan2(
            np.sin(current - previous),
            np.cos(current - previous),
        )
        return angle_difference / dt

    def publish_tip_positions(self, stamp, base_frame_id, boundary_tip, unscaled_tip):
        """Publish meter-valued fitted endpoints from one successful image.

        Boundary tip matches the length-constrained RViz curve. Unscaled tip
        is the fitted endpoint before that hardware-length constraint, not a
        raw depth pixel. Neither is computed again from filtered joint angles.
        """
        positions = [np.asarray(point, dtype=float) for point in (boundary_tip, unscaled_tip)]
        if not base_frame_id or any(
                point.shape != (3,) or not np.all(np.isfinite(point)) for point in positions):
            raise ValueError('Tip positions require a base frame and finite XYZ triples.')
        for publisher, position in zip(
                (self.tip_position_publisher, self.tip_position_unscaled_publisher), positions):
            message = PointStamped()
            message.header.stamp = stamp
            message.header.frame_id = base_frame_id
            message.point = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
            publisher.publish(message)

    def publish_segment_angles(
            self,
            stamp,
            base_frame_id,
            pan_relative,
            pan_absolute,
            tilt_relative,
            tilt_absolute,
            pan_relative_velocity=None,
            tilt_relative_velocity=None):
        """Publish Base-to-tip pan/tilt angles and angular velocities."""
        angle_arrays = {
            'pan_relative': np.asarray(pan_relative, dtype=float),
            'pan_absolute': np.asarray(pan_absolute, dtype=float),
            'tilt_relative': np.asarray(tilt_relative, dtype=float),
            'tilt_absolute': np.asarray(tilt_absolute, dtype=float),
        }
        stamp_sec = self.stamp_to_seconds(stamp)
        velocities = {
            name: np.zeros_like(values)
            for name, values in angle_arrays.items()
        }

        if (self.previous_angle_arrays is not None and
                self.previous_angle_stamp_sec is not None):
            dt = stamp_sec - self.previous_angle_stamp_sec
            if dt > 1e-6:
                velocities = {
                    name: self.wrapped_angular_velocity(
                        values,
                        self.previous_angle_arrays[name],
                        dt,
                    )
                    for name, values in angle_arrays.items()
                }

        # A constant-velocity Kalman state estimates q and q_dot together.
        # Absolute velocities still use wrapped finite differences because
        # they describe fixed-Base direction angles, not individual joints.
        if pan_relative_velocity is not None:
            velocities['pan_relative'] = np.asarray(
                pan_relative_velocity, dtype=float
            )
        if tilt_relative_velocity is not None:
            velocities['tilt_relative'] = np.asarray(
                tilt_relative_velocity, dtype=float
            )

        message = SegmentAngle()
        message.header.stamp = stamp
        message.header.frame_id = base_frame_id
        message.pan_relative = angle_arrays['pan_relative'].tolist()
        message.pan_absolute = angle_arrays['pan_absolute'].tolist()
        message.tilt_relative = angle_arrays['tilt_relative'].tolist()
        message.tilt_absolute = angle_arrays['tilt_absolute'].tolist()
        message.pan_angular_velocity_relative = (
            velocities['pan_relative'].tolist()
        )
        message.pan_angular_velocity_absolute = (
            velocities['pan_absolute'].tolist()
        )
        message.tilt_angular_velocity_relative = (
            velocities['tilt_relative'].tolist()
        )
        message.tilt_angular_velocity_absolute = (
            velocities['tilt_absolute'].tolist()
        )
        self.segment_angle_publisher.publish(message)

        self.previous_angle_arrays = {
            name: values.copy() for name, values in angle_arrays.items()
        }
        self.previous_angle_stamp_sec = stamp_sec

        if not self.published_first_segment_angle:
            self.get_logger().info(
                'Publishing estimated_segment_angle with '
                f'{len(angle_arrays["pan_relative"])}-joint pan/tilt '
                'angles in radians and angular velocities in radians/second.'
            )
            self.published_first_segment_angle = True

    def process(self):

        while not self.shutdown_event.is_set() and rclpy.ok():
            try:
                if not self.frame_event.wait(timeout=0.1):
                    continue
                self.frame_event.clear()
                if self.shutdown_event.is_set() or not rclpy.ok():
                    break

                with self.frame_lock:
                    ready = (
              self.current_frame_flag
              and self.current_frame_ROI is not None
              and self.current_depth is not None
              and self.camera_intrinsics is not None
          )
                    if ready:
                        color_roi = self.current_frame_ROI
                        color_encoding = self.current_frame_encoding
                        depth_roi = self.current_depth
                        color_stamp = self.color_stamp
                        depth_stamp = self.depth_stamp
                        depth_scale = self.depth_scale
                        camera_intrinsics = self.camera_intrinsics.copy()
                        camera_frame_id = (
                self.camera_frame_id or 'camera_color_optical_frame'
            )
                        self.current_frame_flag = False

                if not ready:
                    continue

                # Allow a nearby latest depth frame, but do not estimate motor
                # feedback indefinitely from a disconnected/stalled depth feed.
                depth_age = abs(self.stamp_to_seconds(color_stamp)
                                - self.stamp_to_seconds(depth_stamp))
                if self.max_depth_age_sec and depth_age > self.max_depth_age_sec:
                    self.failed_frame_count += 1
                    self.get_logger().warning(
                        f'Depth/color timestamp gap {depth_age * 1e3:.0f} ms; '
                        'waiting for recent aligned depth.', throttle_duration_sec=2.0)
                    continue

                # Only this worker owns RBSC. Never hold the handoff lock while
                # computing: visualization must be able to take the last result
                # even when the next frame is already being processed.
                success = self.rbsc.postprocess(
                    color_roi,
                    color_encoding=color_encoding,
                    depth_image=depth_roi,
                    camera_intrinsics=camera_intrinsics,
                    depth_scale=depth_scale,
                    roi_offset=(self.roi_x, self.roi_y),
                    timestamp_sec=self.stamp_to_seconds(color_stamp),
                )
                if success is None:
                    self.failed_frame_count += 1
                    self.get_logger().warning(
                        self.rbsc.last_error or 'Reconstruction failed.',
                        throttle_duration_sec=2.0)
                    continue
                if self.shutdown_event.is_set() or not rclpy.ok():
                    break

                rbsc = self.rbsc
                self.publish_base_transform(
                    color_stamp,
                    camera_frame_id,
                    rbsc.base_frame_id,
                    rbsc.base_origin_camera_xyz,
                    rbsc.quaternion_camera_from_base_xyzw,
                )
                self.publish_estimated_segment_transforms(
                    color_stamp,
                    rbsc.base_frame_id,
                    rbsc.segment_center_points_xyz,
                    rbsc.segment_center_frame_rotations_filtered,
                )
                self.publish_segment_angles(
                    color_stamp,
                    rbsc.base_frame_id,
                    rbsc.pan_relative_rad,
                    rbsc.pan_absolute_rad,
                    rbsc.tilt_relative_rad,
                    rbsc.tilt_absolute_rad,
                    (rbsc.pan_relative_velocity_rad_s
                     if rbsc.joint_kalman_filter_enabled else None),
                    (rbsc.tilt_relative_velocity_rad_s
                     if rbsc.joint_kalman_filter_enabled else None),
                )
                # Telemetry must not depend on RViz subscriptions/visualization
                # timing, and failed reconstructions must not repeat old tips.
                self.publish_tip_positions(
                    color_stamp,
                    rbsc.base_frame_id,
                    rbsc.segment_points_xyz[-1],
                    rbsc.tip_position_unscaled_xyz,
                )

                self.processed_frame_count += 1
                self.last_processing_ms = rbsc.last_stage_times_ms['total']
                self.last_result_age_ms = (
                    self.get_clock().now().nanoseconds * 1e-9
                    - self.stamp_to_seconds(color_stamp)
                ) * 1e3
                stage_times_ms = rbsc.last_stage_times_ms
                if (self.debug_timing_enabled and
                        self.timing_log_interval_frames > 0 and
                        self.processed_frame_count %
                        self.timing_log_interval_frames == 0):
                    self.get_logger().info(
                        'RBSC timing [ms] '
                        f"preprocess={stage_times_ms['preprocess']:.1f} "
                        f"(color={stage_times_ms['color_mask']:.1f}, "
                        f"body={stage_times_ms['body_mask']:.1f}, "
                        f"skeleton={stage_times_ms['skeleton']:.1f}, "
                        f"coords={stage_times_ms['coordinates']:.1f}), "
                        f"curve_2d={stage_times_ms['curve_2d']:.1f}, "
                        f"deprojection={stage_times_ms['deprojection']:.1f}, "
                        f"reconstruction_3d="
                        f"{stage_times_ms['reconstruction_3d']:.1f}, "
                        f"total={stage_times_ms['total']:.1f}"
                    )

                self.queue_visualization(color_stamp, camera_frame_id, color_encoding)
            except Exception as e:
                if not self.shutdown_event.is_set() and rclpy.ok():
                    self.get_logger().error(
                        f'process() function exception error : {e}',
                        throttle_duration_sec=2.0,
                    )

    def report_rates(self):
        """Report actual input/success rates even when reconstruction fails."""
        now = time.monotonic()
        elapsed = now - self.rate_window_start
        color = self.color_received_count
        depth = self.depth_received_count
        processed = self.processed_frame_count
        self.get_logger().info(
            'Internal rate [Hz] '
            f'color={(color - self.rate_window_color_count) / elapsed:.1f}, '
            f'aligned_depth={(depth - self.rate_window_depth_count) / elapsed:.1f}, '
            f'processed={(processed - self.rate_window_processed_count) / elapsed:.1f}; '
            f'last compute={self.last_processing_ms:.1f} ms, '
            f'source-to-angle={self.last_result_age_ms:.1f} ms; '
            f'failed_total={self.failed_frame_count}, '
            f'overwritten_input_total={self.overwritten_frame_count}'
        )
        self.rate_window_start = now
        self.rate_window_color_count = color
        self.rate_window_depth_count = depth
        self.rate_window_processed_count = processed

    def queue_visualization(self, stamp, camera_frame_id, encoding):
        """Snapshot one successful frame; replace rather than queue old frames."""
        packet = dict(stamp=stamp, camera_frame_id=camera_frame_id,
                      encoding=encoding, base_frame_id=self.rbsc.base_frame_id)
        if self.segment_skeleton_image_publisher.get_subscription_count():
            packet['overlay'] = self.rbsc.image.copy()
            packet['extended_yx_coords'] = self.rbsc.extended_yx_coords.copy()
        if self.segment_body_binary_image_publisher.get_subscription_count():
            packet['body_image'] = self.rbsc.body_image.copy()
        if self.centerline_points_publisher.get_subscription_count():
            packet['points_xyz'] = self.rbsc.points_xyz.copy()
        if self.reconstruction_markers_publisher.get_subscription_count():
            for name in (
                    'curve_dense_xyz', 'segment_points_xyz', 'segment_tangents_xyz',
                    'segment_center_points_xyz', 'segment_center_tangents_xyz',
                    'segment_directions_xyz', 'segment_directions_projected_xyz',
                    'segment_directions_filtered_xyz'):
                packet[name] = getattr(self.rbsc, name).copy()
        with self.result_lock:
            self.latest_visualization = packet
            self.event.set()

    def publish_crop_depth(self, packet):
        """Preserve raw depth values and source time; no colorization or rescale."""
        pixels, header, encoding, scale, shape, source_info = packet
        try:
            info, metadata = crop_depth_calibration(
                source_info, header, encoding, scale, shape,
                (self.roi_x, self.roi_y, self.roi_w, self.roi_h))
            message = self.br_rgb.cv2_to_imgmsg(pixels, encoding)
            message.header = header
            # Topic ordering is not guaranteed; the recorder joins by exact
            # depth source stamp/frame, with a bounded calibration wait.
            self.crop_depth_metadata_publisher.publish(String(data=json.dumps(metadata)))
            self.crop_depth_info_publisher.publish(info)
            self.crop_depth_publisher.publish(message)
        except (ValueError, TypeError) as error:
            self.get_logger().warning(
                f'Crop depth archive unavailable: {error}', throttle_duration_sec=2.0)

    def realtime_show(self):
        self.get_logger().info('Waiting the first curvefit process...')
        first_result = True

        while not self.shutdown_event.is_set() and rclpy.ok():
            if not self.event.wait(timeout=0.1):
                continue
            # Take-and-clear under the same short lock as producers. A crop
            # wakeup must not publish the previous reconstruction a second time.
            with self.result_lock:
                self.event.clear()
                crop = self.latest_crop
                crop_depth = getattr(self, 'latest_crop_depth', None)
                packet = self.latest_visualization
                self.latest_crop = None
                self.latest_crop_depth = None
                self.latest_visualization = None
            if self.shutdown_event.is_set() or not rclpy.ok():
                break

            try:
                if crop_depth is not None:
                    self.publish_crop_depth(crop_depth)
                if crop is not None:
                    crop_image_msg = self.br_rgb.cv2_to_imgmsg(crop[0], crop[2])
                    crop_image_msg.header = crop[1]
                    self.segment_crop_image_publisher.publish(crop_image_msg)
                if packet is None:
                    continue
                publish_skeleton = 'overlay' in packet
                publish_body = 'body_image' in packet
                publish_cloud = 'points_xyz' in packet
                publish_markers = 'curve_dense_xyz' in packet
                if not any((
                        publish_skeleton,
                        publish_body,
                        publish_cloud,
                        publish_markers)):
                    continue

                visualization_start = time.perf_counter()
                result_stamp = packet['stamp']
                result_frame_id = packet['camera_frame_id']
                result_image_encoding = packet['encoding']
                result_geometry_frame_id = packet['base_frame_id']
                visualization_copy_done = time.perf_counter()
                if first_result:
                    self.get_logger().info(
              'First 3D curve reconstruction completed; visualization started.')
                    first_result = False

                if publish_skeleton or publish_body:
                    image_header = Header()
                    image_header.stamp = result_stamp
                    image_header.frame_id = result_frame_id
                if publish_skeleton:
                    overlay = packet['overlay']
                    curve_pixels_xy = packet['extended_yx_coords'][:, [1, 0]].astype(
                        np.int32, copy=False
                    )
                    cv2.polylines(
                        overlay,
                        [curve_pixels_xy],
                        False,
                        (200, 100, 150),
                        2,
                    )
                    skeleton_image_msg = self.br_rgb.cv2_to_imgmsg(
                        overlay, result_image_encoding
                    )
                    skeleton_image_msg.header = image_header
                    self.segment_skeleton_image_publisher.publish(
                        skeleton_image_msg
                    )
                if publish_body:
                    segment_body_binary_image_msg = self.br_rgb.cv2_to_imgmsg(
                        packet['body_image'], 'mono8'
                    )
                    segment_body_binary_image_msg.header = image_header
                    self.segment_body_binary_image_publisher.publish(
                        segment_body_binary_image_msg
                    )
                visualization_images_done = time.perf_counter()

                if publish_cloud or publish_markers:
                    geometry_header = Header()
                    geometry_header.stamp = result_stamp
                    geometry_header.frame_id = result_geometry_frame_id
                if publish_cloud:
                    centerline_msg = point_cloud2.create_cloud_xyz32(
                        geometry_header,
                        packet['points_xyz'].astype('float32').tolist(),
                    )
                    self.centerline_points_publisher.publish(centerline_msg)
                visualization_cloud_done = time.perf_counter()
                if publish_markers:
                    self.reconstruction_markers_publisher.publish(
                        self.make_reconstruction_markers(
                            geometry_header,
                            packet['curve_dense_xyz'],
                            packet['segment_points_xyz'],
                            packet['segment_tangents_xyz'],
                            packet['segment_center_points_xyz'],
                            packet['segment_center_tangents_xyz'],
                            packet['segment_directions_xyz'],
                            packet['segment_directions_projected_xyz'],
                            packet['segment_directions_filtered_xyz'],
                        )
                    )
                visualization_done = time.perf_counter()
                self.visualized_frame_count += 1
                if (self.debug_timing_enabled and
                        self.timing_log_interval_frames > 0 and
                        self.visualized_frame_count %
                        self.timing_log_interval_frames == 0):
                    self.get_logger().info(
                        'Visualization timing [ms] '
                        f'copy={(visualization_copy_done - visualization_start) * 1e3:.1f}, '
                        f'images={(visualization_images_done - visualization_copy_done) * 1e3:.1f}, '
                        f'cloud={(visualization_cloud_done - visualization_images_done) * 1e3:.1f}, '
                        f'markers={(visualization_done - visualization_cloud_done) * 1e3:.1f}, '
                        f'total={(visualization_done - visualization_start) * 1e3:.1f}'
                    )
            except Exception as e:
                if not self.shutdown_event.is_set() and rclpy.ok():
                    self.get_logger().warning(
                        f'Visualization error: {e}', throttle_duration_sec=2.0)

    def stop_workers(self):
        self.shutdown_event.set()
        self.event.set()
        self.frame_event.set()
        # ros2 launch can forward SIGINT while this process is already handling
        # the terminal SIGINT. Do not let that second interrupt turn a clean
        # worker shutdown into a traceback.
        try:
            self.segment_estimation_thread.join(timeout=2.0)
            self.realtime_show_thread.join(timeout=2.0)
        except KeyboardInterrupt:
            pass

    @staticmethod
    def xyz_to_point(xyz):
        point = Point()
        point.x = float(xyz[0])
        point.y = float(xyz[1])
        point.z = float(xyz[2])
        return point

    def make_reconstruction_markers(
      self,
      header,
      curve_dense_xyz,
      segment_points_xyz,
      segment_tangents_xyz,
      segment_center_points_xyz,
      segment_center_tangents_xyz,
      segment_directions_xyz,
      segment_directions_projected_xyz,
      segment_directions_filtered_xyz):
        markers = MarkerArray()

        # Stable (namespace, id) updates allow RViz to reuse each Ogre object.
        # DELETEALL every frame forces ~100 arrows to be destroyed/recreated.
        if self.previous_marker_keys is None:
            clear = Marker()
            clear.header = header
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)

        fitted_curve = Marker()
        fitted_curve.header = header
        fitted_curve.ns = 'fitted_curve'
        fitted_curve.id = 0
        fitted_curve.type = Marker.LINE_STRIP
        fitted_curve.action = Marker.ADD
        fitted_curve.pose.orientation.w = 1.0
        fitted_curve.scale.x = 0.0005
        fitted_curve.color.r = 0.1
        fitted_curve.color.g = 1.0
        fitted_curve.color.b = 0.1
        fitted_curve.color.a = 1.0
        if len(curve_dense_xyz) > self.visualization_curve_max_points:
            curve_indices = np.linspace(
                0,
                len(curve_dense_xyz) - 1,
                self.visualization_curve_max_points,
                dtype=np.int64,
            )
            curve_dense_xyz = curve_dense_xyz[curve_indices]
        fitted_curve.points = [
        self.xyz_to_point(xyz) for xyz in curve_dense_xyz
    ]
        markers.markers.append(fitted_curve)

        reconstructed_segments = Marker()
        reconstructed_segments.header = header
        reconstructed_segments.ns = 'reconstructed_segments'
        reconstructed_segments.id = 1
        reconstructed_segments.type = Marker.LINE_STRIP
        reconstructed_segments.action = Marker.ADD
        reconstructed_segments.pose.orientation.w = 1.0
        reconstructed_segments.scale.x = 0.0008
        reconstructed_segments.color.r = 0.1
        reconstructed_segments.color.g = 0.3
        reconstructed_segments.color.b = 1.0
        reconstructed_segments.color.a = 1.0
        reconstructed_segments.points = [
        self.xyz_to_point(xyz) for xyz in segment_points_xyz
    ]
        markers.markers.append(reconstructed_segments)

        boundary_points = Marker()
        boundary_points.header = header
        boundary_points.ns = 'segment_boundary_points'
        boundary_points.id = 2
        boundary_points.type = Marker.SPHERE_LIST
        boundary_points.action = Marker.ADD
        boundary_points.pose.orientation.w = 1.0
        boundary_points.scale.x = 0.001
        boundary_points.scale.y = 0.001
        boundary_points.scale.z = 0.001
        boundary_points.color.r = 1.0
        boundary_points.color.g = 0.1
        boundary_points.color.b = 0.1
        boundary_points.color.a = 1.0
        boundary_points.points = [
        self.xyz_to_point(xyz) for xyz in segment_points_xyz
    ]
        markers.markers.append(boundary_points)

        center_points = Marker()
        center_points.header = header
        center_points.ns = 'segment_center_points'
        center_points.id = 3
        center_points.type = Marker.SPHERE_LIST
        center_points.action = Marker.ADD
        center_points.pose.orientation.w = 1.0
        center_points.scale.x = 0.0012
        center_points.scale.y = 0.0012
        center_points.scale.z = 0.0012
        center_points.color.r = 1.0
        center_points.color.g = 0.1
        center_points.color.b = 1.0
        center_points.color.a = 1.0
        center_points.points = [
        self.xyz_to_point(xyz) for xyz in segment_center_points_xyz
    ]
        markers.markers.append(center_points)

        # Marker has no ARROW_LIST type, so publish one ARROW per vector while
        # preserving the existing namespaces for RViz visibility controls.
        vector_marker_sets = (
            (
                'curve_tangents',
                100,
                (1.0, 0.85, 0.0),
                segment_points_xyz,
                segment_tangents_xyz,
                0.006,
            ),
            (
                'segment_center_tangents',
                200,
                (0.0, 1.0, 1.0),
                segment_center_points_xyz,
                segment_center_tangents_xyz,
                0.006,
            ),
            (
                'raw_segment_directions',
                300,
                (0.8, 0.8, 0.8),
                segment_center_points_xyz,
                segment_directions_xyz,
                0.003,
            ),
            (
                'projected_segment_directions',
                400,
                (1.0, 0.35, 0.0),
                segment_center_points_xyz,
                segment_directions_projected_xyz,
                0.003,
            ),
            (
                'filtered_segment_directions',
                500,
                (0.2, 1.0, 0.2),
                segment_center_points_xyz,
                segment_directions_filtered_xyz,
                0.003,
            ),
        )
        for namespace, marker_id, color, starts, directions, length in (
                vector_marker_sets):
            for vector_index, (start, direction) in enumerate(zip(
                    starts, directions)):
                vector_marker = Marker()
                vector_marker.header = header
                vector_marker.ns = namespace
                vector_marker.id = marker_id + vector_index
                vector_marker.type = Marker.ARROW
                vector_marker.action = Marker.ADD
                vector_marker.pose.orientation.w = 1.0
                # ARROW points mode: x=shaft diameter, y=head diameter,
                # z=head length. Keep these narrow relative to 3--6 mm arrows.
                vector_marker.scale.x = 0.0003
                vector_marker.scale.y = 0.0006
                vector_marker.scale.z = 0.0008
                vector_marker.color.r = color[0]
                vector_marker.color.g = color[1]
                vector_marker.color.b = color[2]
                vector_marker.color.a = 1.0
                vector_marker.points = [
                    self.xyz_to_point(start),
                    self.xyz_to_point(start + length * direction),
                ]
                markers.markers.append(vector_marker)

        current_keys = {(m.ns, m.id) for m in markers.markers if m.action == Marker.ADD}
        for namespace, marker_id in (self.previous_marker_keys or set()) - current_keys:
            removed = Marker()
            removed.header = header
            removed.ns = namespace
            removed.id = marker_id
            removed.action = Marker.DELETE
            markers.markers.append(removed)
        self.previous_marker_keys = current_keys
        return markers


def main(args=None):
    configure_image_transport()
    rclpy.init(args=args)
    estimator = SegmentEstimationNode()
    # Image callbacks use the default mutually-exclusive callback group, while
    # heavy reconstruction runs in its own worker thread. A single ROS callback
    # worker avoids needless Python GIL contention from an oversized executor.
    executor = SingleThreadedExecutor()
    executor.add_node(estimator)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch forwards SIGINT to child processes after its own terminal
        # handler runs. Ignore any duplicate SIGINT while resources are being
        # destroyed so cleanup itself cannot be interrupted.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        estimator.stop_workers()
        executor.shutdown()
        estimator.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
