import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy

from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from estimation_pkg.postprocess import RBSC
# from postprocess import RBSC

import threading
import os
import cv2
import json
import time

print(f"[segment] Current Working Directory : {os.getcwd()}")

class SegmentEstimationNode(Node):
    def __init__(self, roi_config_file='config_ROI_ref.json'):
        self.roi_config = self.load_config(roi_config_file)
        self.roi_x = self.roi_config['x']
        self.roi_y = self.roi_config['y']
        self.roi_w = self.roi_config['w']
        self.roi_h = self.roi_config['h']
        self.current_frame = None
        self.current_frame_ROI = None
        self.current_depth = None
        self.color_stamp = None
        self.depth_stamp = None
        self.depth_scale = None
        self.camera_intrinsics = None
        self.camera_frame_id = None
        self.result_stamp = None
        self.result_frame_id = None
        self.received_first_color = False
        self.received_first_depth = False
        self.received_first_camera_info = False
        self.frame_lock = threading.Lock()
        self.result_lock = threading.Lock()
        self.rbsc = RBSC()

        # self.joint_angle = np.array(0)
        print("============================")
        print(os.getcwd())
        print(os.path.abspath(__file__))
        print("============================")

        super().__init__('segment_estimation_node')
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
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        QOS_OUTPUT_LATEST = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        QOS_MARKER_LATEST = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
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
      QOS_OUTPUT_LATEST
    )
        self.segment_body_binary_image_publisher = self.create_publisher(
      Image,
      'estimated_segment_body_binary_image',
      QOS_OUTPUT_LATEST
    )
        self.segment_skeleton_image_publisher = self.create_publisher(
      Image,
      'estimated_segment_skeleton_image',
      QOS_OUTPUT_LATEST
    )
        self.centerline_points_publisher = self.create_publisher(
      PointCloud2,
      'estimated_segment_centerline_points',
      QOS_OUTPUT_LATEST
    )
        self.reconstruction_markers_publisher = self.create_publisher(
      MarkerArray,
      'estimated_segment_reconstruction_markers',
      QOS_MARKER_LATEST
    )

        self.segment_estimation_thread = threading.Thread(
        target=self.process, daemon=True)
        self.realtime_show_thread = threading.Thread(
        target=self.realtime_show, daemon=True)
        self.event = threading.Event()

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

    def color_image_rect_raw_callback(self, data):
        frame = self.br_rgb.imgmsg_to_cv2(data, 'bgr8')
        color_roi = frame[
        self.roi_y:self.roi_y + self.roi_h,
        self.roi_x:self.roi_x + self.roi_w,
    ]
        crop_image_msg = self.br_rgb.cv2_to_imgmsg(color_roi, 'bgr8')
        crop_image_msg.header = data.header
        self.segment_crop_image_publisher.publish(crop_image_msg)

        if not self.received_first_color:
            self.get_logger().info(
          f'First color frame received; ROI shape={color_roi.shape}.')
            self.received_first_color = True

        with self.frame_lock:
            self.current_frame_flag = True
            self.color_stamp = data.header.stamp
            self.current_frame = frame
            self.current_frame_ROI = color_roi

    def aligned_depth_callback(self, data):
        depth = self.br_rgb.imgmsg_to_cv2(data, 'passthrough')
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
            self.current_depth = depth
            self.depth_stamp = data.header.stamp
            self.depth_scale = depth_scale

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

    def process(self):

        while rclpy.ok():
            try:
                with self.frame_lock:
                    ready = (
              self.current_frame_flag
              and self.current_frame is not None
              and self.current_depth is not None
              and self.camera_intrinsics is not None
          )
                    if ready:
                        color = self.current_frame.copy()
                        depth = self.current_depth.copy()
                        color_stamp = self.color_stamp
                        depth_scale = self.depth_scale
                        camera_intrinsics = self.camera_intrinsics.copy()
                        camera_frame_id = (
                self.camera_frame_id or 'camera_color_optical_frame'
            )
                        self.current_frame_flag = False

                if not ready:
                    time.sleep(0.005)
                    continue

                color_roi = color[
            self.roi_y:self.roi_y + self.roi_h,
            self.roi_x:self.roi_x + self.roi_w,
        ]
                depth_roi = depth[
            self.roi_y:self.roi_y + self.roi_h,
            self.roi_x:self.roi_x + self.roi_w,
        ]
                self.current_frame_ROI = color_roi

                with self.result_lock:
                    success = self.rbsc.postprocess(
              color_roi,
              depth_image=depth_roi,
              camera_intrinsics=camera_intrinsics,
              depth_scale=depth_scale,
              roi_offset=(self.roi_x, self.roi_y),
          )
                    if success is not None:
                        self.result_stamp = color_stamp
                        self.result_frame_id = camera_frame_id
                if success is None:
                    continue

                # Pan/tilt angles are intentionally not published here yet. This stage
                # produces the ordered 3D centerline and 18 segment directions.
                self.event.set()
            except Exception as e:
                self.get_logger().error(f'process() function exception error : {e}')

    def realtime_show(self):
        self.get_logger().info('Waiting the first curvefit process...')
        first_result = True

        while rclpy.ok():
            if not self.event.wait(timeout=0.1):
                continue
            self.event.clear()
            try:
                with self.result_lock:
                    overlay = self.rbsc.image.copy()
                    body_image = self.rbsc.body_image.copy()
                    extended_yx_coords = self.rbsc.extended_yx_coords.copy()
                    points_xyz = self.rbsc.points_xyz.copy()
                    curve_dense_xyz = self.rbsc.curve_dense_xyz.copy()
                    segment_points_xyz = self.rbsc.segment_points_xyz.copy()
                    segment_tangents_xyz = self.rbsc.segment_tangents_xyz.copy()
                    result_stamp = self.result_stamp
                    result_frame_id = self.result_frame_id
                if first_result:
                    self.get_logger().info(
              'First 3D curve reconstruction completed; visualization started.')
                    first_result = False
                for y,x in extended_yx_coords:
                    cv2.circle(overlay, (int(x), int(y)), 1, (100,200,255), -1)

                header = Header()
                header.stamp = result_stamp
                header.frame_id = result_frame_id

                skeleton_image_msg = self.br_rgb.cv2_to_imgmsg(overlay, 'bgr8')
                skeleton_image_msg.header = header
                self.segment_skeleton_image_publisher.publish(skeleton_image_msg)

                segment_body_binary_image_msg = self.br_rgb.cv2_to_imgmsg(
            body_image, 'mono8')
                segment_body_binary_image_msg.header = header
                self.segment_body_binary_image_publisher.publish(
            segment_body_binary_image_msg)

                centerline_msg = point_cloud2.create_cloud_xyz32(
            header,
            points_xyz.astype('float32').tolist(),
        )
                self.centerline_points_publisher.publish(centerline_msg)
                self.reconstruction_markers_publisher.publish(
            self.make_reconstruction_markers(
                header,
                curve_dense_xyz,
                segment_points_xyz,
                segment_tangents_xyz,
            )
        )
            except Exception as e:
                self.get_logger().warning(f'Visualization error: {e}')

        # Cleanup
        cv2.destroyAllWindows()

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
      segment_tangents_xyz):
        markers = MarkerArray()

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
        fitted_curve.scale.x = 0.0008
        fitted_curve.color.r = 0.1
        fitted_curve.color.g = 1.0
        fitted_curve.color.b = 0.1
        fitted_curve.color.a = 1.0
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
        reconstructed_segments.scale.x = 0.0012
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
        boundary_points.scale.x = 0.0025
        boundary_points.scale.y = 0.0025
        boundary_points.scale.z = 0.0025
        boundary_points.color.r = 1.0
        boundary_points.color.g = 0.1
        boundary_points.color.b = 0.1
        boundary_points.color.a = 1.0
        boundary_points.points = [
        self.xyz_to_point(xyz) for xyz in segment_points_xyz
    ]
        markers.markers.append(boundary_points)

        tangent_length = 0.006
        for index, (start, direction) in enumerate(zip(
        segment_points_xyz,
        segment_tangents_xyz,
    )):
            tangent = Marker()
            tangent.header = header
            tangent.ns = 'curve_tangents'
            tangent.id = 100 + index
            tangent.type = Marker.ARROW
            tangent.action = Marker.ADD
            tangent.pose.orientation.w = 1.0
            tangent.scale.x = 0.0007
            tangent.scale.y = 0.0016
            tangent.scale.z = 0.0020
            tangent.color.r = 1.0
            tangent.color.g = 0.85
            tangent.color.b = 0.0
            tangent.color.a = 1.0
            tangent.points = [
          self.xyz_to_point(start),
          self.xyz_to_point(start + tangent_length * direction),
      ]
            markers.markers.append(tangent)

        return markers


def main(args=None):
  rclpy.init(args=args)
  try:
    estimator = SegmentEstimationNode()
    executor = MultiThreadedExecutor()
    #  executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(estimator)
    try:
      executor.spin()
    except KeyboardInterrupt:
      estimator.get_logger().warning('Keyboard Interrupt (SIGINT)')
    finally:
      executor.shutdown()
      estimator.destroy_node()
  finally:
     rclpy.shutdown()

if __name__ == '__main__':
  main()
