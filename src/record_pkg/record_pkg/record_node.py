import sys, os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
import rosbag2_py._storage
from std_srvs.srv import SetBool
from ros2bag.api import rosbag2_py
import rosbag2_py
from rclpy.serialization import serialize_message

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from custom_interfaces.msg import LoadcellState
from custom_interfaces.msg import MotorCommand
from custom_interfaces.msg import MotorState
from custom_interfaces.msg import DynamicMIMOValues
from custom_interfaces.srv import MoveMotorDirect
from custom_interfaces.srv import MoveToolAngle

from rclpy.executors import MultiThreadedExecutor

print("Python executable:", sys.executable)
print("Python version:", sys.version)

# import mediapipe as mp
# import sympy
# import json
# print("Hi, JSON")
# import xacro
# import sklearn

import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from datetime import datetime
import PIL

import csv
import json
import subprocess   # CLI

class RecordNode(Node):
    def __init__(self):
        """
        ROS2 bag profile set
        """
        super().__init__('record_node')
        self.create_service(SetBool, '/data/record', self.record_callback)
        self.is_recording = False
        self.data_count = 0
        self.data_count_dMv = 0

        # self.rosbag_writer = rosbag2_py.SequentialWriter()
        # storage_options = rosbag2_py._storage.StorageOptions(
        #     uri='SIFM_bag',
        #     storage_id='sqlite3')
        # converter_options = rosbag2_py._storage.ConverterOptions('', '')
        # self.rosbag_writer.open(storage_options, converter_options)

        # topic_info_fts_data = rosbag2_py._storage.TopicMetadata(
        #     # id=0,
        #     name='fts_data',
        #     type='geometry_msgs/msg/WrenchStamped',
        #     serialization_format='cdr')
        # topic_info_loadcell = rosbag2_py._storage.TopicMetadata(
        #     # id=0,
        #     name='loadcell_state',
        #     type='custom_interfaces/msg/LoadcellState',
        #     serialization_format='cdr')
        # topic_info_motor = rosbag2_py._storage.TopicMetadata(
        #     # id=0,
        #     name='motor_state',
        #     type='custom_interfaces/msg/MotorState',
        #     serialization_format='cdr')
        # topic_info_image = rosbag2_py._storage.TopicMetadata(
        #     # id=0,
        #     name='camera/color/image_rect_raw',
        #     type='sensor_msgs/msg/Image',
        #     serialization_format='cdr')
        
        # self.rosbag_writer.create_topic(topic_info_fts_data)
        # self.rosbag_writer.create_topic(topic_info_loadcell)
        # self.rosbag_writer.create_topic(topic_info_motor)
        # self.rosbag_writer.create_topic(topic_info_image)

        # ROS2 topic subscriber
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.motor_command_publisher_ = self.create_publisher(MotorCommand, 'motor_command', QOS_RKL10V)
        
        self.fts_data_flag = False
        self.fts_data = WrenchStamped()
        self.fts_subscriber = self.create_subscription(
            WrenchStamped,
            'fts_data',
            self.read_fts_data,
            QOS_RKL10V
        )
        self.fts_data_offset = WrenchStamped()
        self.fts_offset_subscriber = self.create_subscription(
            WrenchStamped,
            'fts_data_offset',
            self.read_fts_data_offset,
            QOS_RKL10V
        )
        self.get_logger().info('fts_data subscriber is created.')

        self.loadcell_data_flag = False
        self.loadcell_data = LoadcellState()
        self.lc_subscriber = self.create_subscription(
            LoadcellState,
            'loadcell_state',
            self.read_loadcell_data,
            QOS_RKL10V
        )
        self.loadcell_data_offset = LoadcellState()
        self.lc_offset_subscriber = self.create_subscription(
            LoadcellState,
            'loadcell_state_offset',
            self.read_loadcell_data_offset,
            QOS_RKL10V
        )
        self.get_logger().info('loadcell_data subscriber is created.')

        self.motor_state_flag = False
        self.motor_state = MotorState()
        self.motor_state_subscriber = self.create_subscription(
            MotorState,
            'motor_state',
            self.read_motor_state,
            QOS_RKL10V
        )
        self.get_logger().info('motor_state subscriber is created.')
        
        self.wire_length_flag = False
        self.wire_length = MotorState()
        self.wire_length_subscriber = self.create_subscription(
            Float32MultiArray,
            'wire_length',
            self.read_wire_length,
            QOS_RKL10V
        )
        self.get_logger().info('wire_length subscriber is created.')

        self.dynamic_MIMO_values_flag = False
        self.dynamic_MIMO_values = DynamicMIMOValues()
        self.dynamic_MIMO_values_subscriber = self.create_subscription(
            DynamicMIMOValues,
            'dynamic_MIMO_values',
            self.read_dynamic_MIMO_values,
            QOS_RKL10V
        )
        self.get_logger().info('dynamic_MIMO_values subscriber is created.')


        # self.realsense_subscriber = RealSenseSubscriber()
        # color rectified image. RGB format
        self.image_flag = False
        self.br_rgb = CvBridge()
        self.color_image_rect_raw_subscriber = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            # "camera/color/image_rect_raw",
            self.color_image_rect_raw_callback,
            1)
        self.get_logger().info('realsense-camera subscriber is created.')

        # estimated image
        self.segment_angle_image_flag = False
        self.segment_angle_image = Image()
        self.segment_angle_image_subscriber = self.create_subscription(
            Image,
            "estimated_segment_angle_image",
            self.segment_angle_image_callback,
            1)
        self.get_logger().info('realsense-camera subscriber is created.')


        ### ================================================================
        ### file managers
        ### ================================================================
        # hw_definition.hpp 파일의 경로 설정
        self.get_logger().info(f'{os.getcwd()}')

        hw_definition_hpp_path = './src/robot_control_pkg/include/robot_control_pkg/hw_definition.hpp'
        # 파싱하여 상수 값을 읽어옴
        constants = self.parse_hw_definition_hpp(hw_definition_hpp_path)
        # 상수 값 출력
        for key, value in constants.items():
            # self.get_logger().info(f'{key}: {value} ({type(key)}/{type(value)})')
            if key == 'NUM_OF_MOTORS':
                self.numofmotors = int(value)
            elif key == 'NUM_OF_JOINT':
                self.numofjoints = int(value)
            elif key == 'SEGMENT_ARC':
                self.segment_arc = float(value)
            elif key == 'SEGMENT_DIAMETER':
                self.segment_dia = float(value)
            elif key == 'WIRE_DISTANCE':
                self.segment_wd = float(value)

            if key == 'OP_MODE':
                if value == '0x08':
                    self.get_logger().info(f'OP_MODE: CSP')
                elif value == '0x09':
                    self.get_logger().info(f'OP_MODE: CSV')
                self.opmode = value

        self.directory_path = None
        self.directory_path_image = None
        self.directory_path_csv = None

    
    ### ================================================================
    ### Functions
    ### ================================================================
        """ROS2 Functions
        - callback functions of subscribers, service_server
        """ 
    def record_callback(self, request, response):
        self.is_recording = request.data

        try:
            if self.is_recording:
                # Start recording
                self.get_logger().info('Start recording')
                # os.chdir(self.directory_path)
                self.create_directory()
                self.create_csv()
                self.create_csv_dynamics_MIMO_values()
                self.create_metadata_json()


                """
                if record all topics excluding /camera/* data, use under line
                """
                cmd = 'ros2 bag record -a --exclude "/camera(.*)"'
                self.bag_process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=self.directory_path)


                """
                if record all(-a) topics, use under code
                """
                # self.bag_process = subprocess.Popen(['ros2',
                #                                     'bag',
                #                                     'record',
                #                                     '-a'], cwd=self.directory_path)

                response.success = True
                response.message = 'Start Recording.'
            elif self.is_recording == False:
                # Subscribe to topics you want to record
                self.get_logger().info('Stop recording')
                self.csv_file.close()
                self.csv_file_dMv.close()
                self.bag_process.terminate()
                response.success = True
                response.message = 'Stop Recording.'
                self.data_count = 0
                self.data_count_dMv = 0
        except Exception as e:
            self.get_logger().info(f'Exception Error as {e}')
            response.success = False
            response.message = 'Error is up for recording.'

        return response

    # def stop_record_callback(self, request, response):
    #     if self.recorder:
    #         # Stop recording
    #         self.get_logger().info('Stop recording')
    #         self.recorder.stop()
    #         self.recorder = None
    #     else:
    #         self.get_logger().info('Not recording')
    #     return response


    
    def color_image_rect_raw_callback(self, data):
        """_summary_

        Args:
            data (_type_): _description_
        """
        # self.get_logger().info("Receiving RGB frame")
        self.image_flag = True
        self.capture_time = data.header.stamp
        self.current_frame = self.br_rgb.imgmsg_to_cv2(data, 'bgr8')

        if self.is_recording:
            self.data_count += 1
            image_file = str(self.data_count) + '_' + str(self.capture_time.sec) + '-' + str(self.capture_time.nanosec) +'.png'
            cv2.imwrite(self.directory_path_image + '/' + image_file, self.current_frame)
            self.update_csv()

        # cv2.imshow("[Record Node] rgb", self.current_frame)
        # cv2.waitKey(1)
        # if self.is_recording:
        #     self.rosbag_writer.write(
        #     'camera/color/image_rect_raw',
        #     serialize_message(data),
        #     self.get_clock().now().nanoseconds
        # )

        # return

    def segment_angle_image_callback(self, data):
        self.segment_angle_image_flag = True
        self.segment_angle_image_capture_time = data.header.stamp
        self.segment_angle_image = self.br_rgb.imgmsg_to_cv2(data, 'bgr8')

    def read_fts_data(self, msg):
        self.fts_data_flag = True
        self.fts_data = msg
    def read_fts_data_offset(self, msg):
        self.fts_data_offset = msg

    def read_loadcell_data(self, msg):
        self.loadcell_data_flag = True
        self.loadcell_data = msg
    def read_loadcell_data_offset(self, msg):
        self.loadcell_data_offset = msg

    def read_motor_state(self, msg):
        self.motor_state_flag = True
        self.motor_state = msg

    def read_wire_length(self, msg):
        self.wire_length_flag = True
        self.wire_length = msg

    def read_dynamic_MIMO_values(self, msg):
        self.dynamic_MIMO_values_flag = True
        self.dynamic_MIMO_values = msg

        if self.is_recording:
            self.data_count_dMv += 1
            image_file = str(self.data_count_dMv) + '_' + str(self.dynamic_MIMO_values.header.stamp.sec) + '-' + str(self.dynamic_MIMO_values.header.stamp.nanosec) +'.png'
            cv2.imwrite(self.directory_path_image_with_estimated_angle + '/' + image_file, self.segment_angle_image)
            self.update_csv_dynamics_MIMO_values()


    def create_directory(self):
        c_time = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.directory_path = os.path.join('./record', c_time)
        if not os.path.exists(self.directory_path):
            os.makedirs(self.directory_path)
            self.get_logger().info(f'Directory is created => {self.directory_path}')
        self.directory_path_image = os.path.join(self.directory_path, 'images')
        self.directory_path_image_with_estimated_angle = os.path.join(self.directory_path, 'images_with_estimated_angle')
        if not os.path.exists(self.directory_path_image):
            os.makedirs(self.directory_path_image)
        if not os.path.exists(self.directory_path_image_with_estimated_angle):
            os.makedirs(self.directory_path_image_with_estimated_angle)
        self.directory_path_csv = self.directory_path
        self.directory_path_csv_dMv = self.directory_path

    ##################################
    def create_csv(self):
        self.csv_file_name = os.path.join(self.directory_path_csv, 'data.csv')
        self.get_logger().info(f'CSV is created => name : {self.csv_file_name}')

        self.csv_headers = {}
        self.csv_headers['sec'] = []
        self.csv_headers['nanosec'] = []
        self.csv_headers['image'] = []
        for i in range(self.numofmotors):
            self.csv_headers[f'motor position #{i}'] = []
        for i in range(self.numofmotors):
            self.csv_headers[f'wire length #{i}'] = []
        for i in range(self.numofmotors):
            self.csv_headers[f'loadcell #{i}'] = []
        self.csv_headers['fx'] = []
        self.csv_headers['fy'] = []
        self.csv_headers['fz'] = []
        self.csv_headers['tx'] = []
        self.csv_headers['ty'] = []
        self.csv_headers['tz'] = []

        self.csv_file = open(self.csv_file_name, mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.csv_headers.keys())
        self.csv_file.flush()

    def update_csv(self):
        # if not self.image_flag and not self.fts_data_flag and not self.motor_state_flag and not self.loadcell_data_flag:
        #     self.get_logger().warning(f'All data are not subscribed')
        #     return
        
        timestamp_sec = str(self.capture_time.sec)
        timestamp_nanosec = str(self.capture_time.nanosec)
        image_file = str(self.data_count) + '_' + str(self.capture_time.sec) + '-' + str(self.capture_time.nanosec) +'.png'
        actual_position = self.motor_state.actual_position
        wire_length = self.wire_length.data
        loadcell_stress = self.loadcell_data.stress
        forcexyz = self.fts_data.wrench.force
        torquexyz = self.fts_data.wrench.torque
        
        self.csv_writer.writerow([timestamp_sec, timestamp_nanosec, image_file]
                                 + [str(value) for value in actual_position]
                                 + [str(value) for value in wire_length]
                                 + [str(value) for value in loadcell_stress]
                                 + [str(forcexyz.x)]
                                 + [str(forcexyz.y)]
                                 + [str(forcexyz.z)]
                                 + [str(torquexyz.x)]
                                 + [str(torquexyz.y)]
                                 + [str(torquexyz.z)])
        self.csv_file.flush()
        pass

    ##################################
    def create_csv_dynamics_MIMO_values(self):
        self.csv_file_name_dMv = os.path.join(self.directory_path_csv_dMv, 'data_DynamicMIMOValues.csv')
        self.get_logger().info(f'CSV is created => name : {self.csv_file_name_dMv}')

        self.csv_headers_dMv = {}
        self.csv_headers_dMv['sec'] = []
        self.csv_headers_dMv['nanosec'] = []
        self.csv_headers_dMv['image'] = []
        self.csv_headers_dMv['sampling_time'] = []
        self.csv_headers_dMv['p_gain'] = []
        self.csv_headers_dMv['i_gain'] = []
        self.csv_headers_dMv['d_gain'] = []
        self.csv_headers_dMv['theta_desired'] = []
        self.csv_headers_dMv['theta_actual'] = []
        self.csv_headers_dMv['omega_actual'] = []
        # self.csv_headers_dMv['segment_theta_relative'] = []
        # self.csv_headers_dMv['segment_omega_relative'] = []
        for i in range(self.numofmotors):
            self.csv_headers_dMv[f'tension #{i}'] = []
        self.csv_headers_dMv[f'external_force_x'] = []
        self.csv_headers_dMv[f'external_force_y'] = []
        self.csv_headers_dMv['external_torque'] = []
        self.csv_headers_dMv['friction_torque'] = []
        self.csv_headers_dMv['damping_coefficient'] = []
        self.csv_headers_dMv['input_alpha'] = []
        self.csv_headers_dMv['input_omega'] = []
        self.csv_headers_dMv['input_theta'] = []

        self.csv_file_dMv = open(self.csv_file_name_dMv, mode='w')
        self.csv_writer_dMv = csv.writer(self.csv_file_dMv)
        self.csv_writer_dMv.writerow(self.csv_headers_dMv.keys())
        self.csv_file_dMv.flush()
    
    def update_csv_dynamics_MIMO_values(self):
        # if not self.image_flag and not self.fts_data_flag and not self.motor_state_flag and not self.loadcell_data_flag:
        #     self.get_logger().warning(f'All data are not subscribed')
        #     return
        timestamp_sec = str(self.dynamic_MIMO_values.header.stamp.sec)
        timestamp_nanosec = str(self.dynamic_MIMO_values.header.stamp.nanosec)
        image_file = str(self.data_count_dMv) + '_' + str(timestamp_sec) + '-' + str(timestamp_nanosec) +'.png'
        
        self.csv_writer_dMv.writerow([timestamp_sec, timestamp_nanosec, image_file]
                                 + [str(self.dynamic_MIMO_values.sampling_time)]
                                 + [str(self.dynamic_MIMO_values.p_gain)]
                                 + [str(self.dynamic_MIMO_values.i_gain)]
                                 + [str(self.dynamic_MIMO_values.d_gain)]
                                 + [str(self.dynamic_MIMO_values.theta_desired)]
                                 + [str(self.dynamic_MIMO_values.theta_actual)]
                                 + [str(self.dynamic_MIMO_values.omega_actual)]
                                 + [str(value) for value in self.dynamic_MIMO_values.tension]
                                 + [str(value) for value in self.dynamic_MIMO_values.external_force]
                                 + [str(self.dynamic_MIMO_values.external_torque)]
                                 + [str(self.dynamic_MIMO_values.friction_torque)]
                                 + [str(self.dynamic_MIMO_values.damping_coefficient)]
                                 + [str(self.dynamic_MIMO_values.input_alpha)]
                                 + [str(self.dynamic_MIMO_values.input_omega)]
                                 + [str(self.dynamic_MIMO_values.input_theta)])
        self.csv_file_dMv.flush()
        pass

    #######################################

    def create_metadata_json(self):
        # declare dictionary of metadata
        metadata = {
            "info": {
                "NUM_OF_MOTORS": self.numofmotors,
                'NUM_OF_JOINT': self.numofjoints,
                'SEGMENT_ARC': self.segment_arc,
                'SEGMENT_DIAMETER': self.segment_dia,
                'WIRE_DISTANCE': self.segment_wd
            },
            "units": {
                "timestamp sec": "s",
                "timestamp nanosec": "ns",
                "image file": "filename",
                "motor position": "encoder inc",
                "wire length": "mm",
                "loadcell": "g",
                "fx": "mN",
                "fy": "mN",
                "fz": "mN",
                "tx": "mNm",
                "ty": "mNm",
                "tz": "mNm"
            },
            "offsets": {
                "fx": self.fts_data_offset.wrench.force.x,
                "fy": self.fts_data_offset.wrench.force.y,
                "fz": self.fts_data_offset.wrench.force.z,
                "tx": self.fts_data_offset.wrench.torque.x,
                "ty": self.fts_data_offset.wrench.torque.y,
                "tz": self.fts_data_offset.wrench.torque.z
            },
        }

        ## Add units of motors and loadcells.
        # for i in range(self.numofmotors):
        #     metadata["units"][f"motor #{i}"] = "encoder inc"
        #     metadata["units"][f"loadcell #{i}"] = "mN"

        # 메타데이터를 JSON 파일로 저장합니다.
        self.metadata_file_name = os.path.join(self.directory_path_csv, "metadata.json")
        with open(self.metadata_file_name, 'w') as metadata_file:
            json.dump(metadata, metadata_file, indent=4)

        self.get_logger().info(f"metadata.json is created => name : {self.metadata_file_name}")

    def parse_hw_definition_hpp(self, file_path):
        """
        Parse the given HPP file and extract global variables and their values.
        Ignore lines starting with "//" and lines containing only whitespace.

        Args:
            file_path (str): The path to the HPP file.

        Returns:
            dict: A dictionary containing global variables and their values.
        """
        constants = {}
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                if line.startswith('#define'):
                    parts = line.split()
                    if len(parts) >= 3:
                        key = parts[1]
                        value = parts[2]
                        constants[key] = value
        return constants
    
def main(args=None):
    # rclpy.init(args=args)
    # record_node = RecordNode()
    # rclpy.spin(record_node)
    # record_node.destroy_node()
    # print('record node is destroyed')
    # rclpy.shutdown()
    # print('rclpy shutdonw')

    rclpy.init(args=args)
    try:
        record_node = RecordNode()
        executor = MultiThreadedExecutor()
        #  executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(record_node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            record_node.get_logger().warning('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            record_node.destroy_node()
            print('record node is destroyed')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
