import sys, os
import threading
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so'

import PyQt5.QtCore
print(PyQt5.QtCore.QLibraryInfo.location(PyQt5.QtCore.QLibraryInfo.PluginsPath))

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QCheckBox
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QTextStream, Qt, QObject, pyqtSignal
from PyQt5 import QtCore
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from collections import deque
import numpy as np

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool
from custom_interfaces.msg import LoadcellState
from custom_interfaces.msg import MotorCommand
from custom_interfaces.msg import MotorState
from custom_interfaces.msg import DataFilterSetting
from custom_interfaces.srv import MoveMotorDirect
from custom_interfaces.srv import MoveToolAngle

# import rs_read
from gui_py_pkg.rs_read import *
# from gui_py_pkg.rs_read import RealSenseSubscriber

import cv2
import numpy as np
from cv_bridge import CvBridge

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')

        # hw_definition.hpp 파일의 경로 설정
        print(os.getcwd())
        hw_definition_hpp_path = './src/robot_control_pkg/include/robot_control_pkg/hw_definition.hpp'
        # 파싱하여 상수 값을 읽어옴
        constants = self.parse_hw_definition_hpp(hw_definition_hpp_path)
        # 상수 값 출력
        self.get_logger().info(f'{PyQt5.QtCore.QLibraryInfo.location(PyQt5.QtCore.QLibraryInfo.PluginsPath)}')

        for key, value in constants.items():
            # self.get_logger().info(f'{key}: {value}')
            self.get_logger().info(f'{key}: {value} ({type(key)}/{type(value)})')
            if key == 'NUM_OF_MOTORS':
                self.numofmotors = int(value)
            if key == 'OP_MODE':
                if value == '0x08':
                    self.get_logger().info(f'OP_MODE: CSP')
                elif value == '0x09':
                    self.get_logger().info(f'OP_MODE: CSV')
                self.opmode = value

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.motor_command_publisher_ = self.create_publisher(
            MotorCommand,
            'motor_command',
            QOS_RKL10V
        )
        
        self.fts_data_flag = False
        self.fts_data = WrenchStamped()
        self.fts_subscriber = self.create_subscription(
            WrenchStamped,
            'fts_data',
            self.read_fts_data,
            QOS_RKL10V
        )
        self.get_logger().info('fts_data subscriber is created.')

        self.loadcell_data = LoadcellState()
        self.loadcell_subscriber = self.create_subscription(
            LoadcellState,
            'loadcell_state',
            self.read_loadcell_data,
            QOS_RKL10V
        )
        self.get_logger().info('loadcell_data subscriber is created.')

        self.motor_state = MotorState()
        self.motor_state_subscriber = self.create_subscription(
            MotorState,
            'motor_state',
            self.read_motor_state,
            QOS_RKL10V
        )
        self.get_logger().info('motor_state subscriber is created.')
        
        self.data_filter_setting_publisher = self.create_publisher(
            DataFilterSetting,
            'data_filter_setting',
            QOS_RKL10V
        ) 
        # self.LPF_state_publisher = self.create_publisher(
        #     Bool,
        #     'LPF_state',
        #     QOS_RKL10V
        # )
        # self.moving_avg_filter_state_publisher = self.create_publisher(
        #     Bool,
        #     'MAF_state',
        #     QOS_RKL10V
        # )

        

        # self.data_received_signal = pyqtSignal()
        # self.data_received_signal = pyqtSignal(bool)

        # self.realsense_subscriber = RealSenseSubscriber()
        # color rectified image. RGB format
        # self.br_rgb = CvBridge()
        # self.color_image_rect_raw_subscriber = self.create_subscription(
        #     Image,
        #     "camera/color/image_rect_raw",
        #     self.color_image_rect_raw_callback,
        #     QOS_RKL10V)
        # self.get_logger().info('realsense-camera subscriber is created.')

        self.move_motor_direct_service_client = self.create_client(
            MoveMotorDirect,
            '/move_motor_direct'
        )
        while not self.move_motor_direct_service_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('The "/move_motor_direct" service server not available. Check the kinematics_control_node')

        self.move_tool_angle_service_client = self.create_client(
            MoveToolAngle,
            '/kinematics/move_tool_angle'
        )
        while not self.move_tool_angle_service_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('The "/kinematics/move_tool_angle" service server not available. Check the kinematics_control_node')

        self.move_tool_angle_dynamics_service_client = self.create_client(
            MoveToolAngle,
            '/dynamics/move_tool_angle'
        )
        while not self.move_tool_angle_dynamics_service_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('The "/dynamics/move_tool_angle" service server not available. Check the kinematics_control_node')

        self.control_mode_client = self.create_client(
            SetBool,
            'control/control_mode'
        )
        while not self.move_tool_angle_dynamics_service_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('The "/dynamics/move_tool_angle" service server not available. Check the kinematics_control_node')

        self.recoder_service_client = self.create_client(
            SetBool,
            '/data/record'
        )

        self.set_zero_client = self.create_client(
            SetBool,
            '/serial_data/set_zero'
        )


        # while not self.recoder_service_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().warning('The "data/recode" service server not available. Check the kinematics_control_node')

    ### ================================================================
    ### Functions
    ### ================================================================
    # def color_image_rect_raw_callback(self, data):
    #     # self.get_logger().info("Receiving RGB frame")
    #     current_frame = self.br_rgb.imgmsg_to_cv2(data, 'bgr8')
    #     cv2.imshow("[GUI Node] rgb", current_frame)
    #     cv2.waitKey(1)
    #     # return

    

    def read_fts_data(self, msg):
        self.fts_data_flag = True
        # self.data_received_signal.emit(True)     # DY
        self.fts_data = msg

    def read_loadcell_data(self, msg):
        self.loadcell_data = msg

    def read_motor_state(self, msg):
        self.motor_state = msg
        # pass

    def publish(self, msg):
        self.motor_command_publisher_.publish(msg)

    def send_request_move_motor_direct(self, idx=0, tp=0, tvp=50):
        service_request = MoveMotorDirect.Request()
        service_request.index_motor = idx
        service_request.target_position = tp
        service_request.target_velocity_profile = tvp
        future = self.move_motor_direct_service_client.call_async(service_request)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_request_move_tool_angle(self, pan=0.0, tilt=0.0, grip=0.0, mode=0):
        service_request = MoveToolAngle.Request()
        service_request.panangle = pan
        service_request.tiltangle = tilt
        service_request.gripangle = grip
        service_request.mode = mode
        future = self.move_tool_angle_service_client.call_async(service_request)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_request_move_tool_angle_dynamics(self, pan=0.0, tilt=0.0, grip=0.0, mode=0):
        service_request = MoveToolAngle.Request()
        service_request.panangle = pan
        service_request.tiltangle = tilt
        service_request.gripangle = grip
        service_request.mode = mode
        future = self.move_tool_angle_dynamics_service_client.call_async(service_request)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    # mode : true-dynamics / false-kinematics
    def send_request_change_control_mode(self, mode=False):
        service_request = SetBool.Request()
        service_request.data = mode
        future = self.control_mode_client.call_async(service_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_request_record_start(self):
        service_request = SetBool.Request()
        service_request.data = True
        future = self.recoder_service_client.call_async(service_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_request_record_stop(self):
        service_request = SetBool.Request()
        service_request.data = False
        future = self.recoder_service_client.call_async(service_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_request_set_zero(self):
        service_request = SetBool.Request()
        service_request.data = True
        future = self.set_zero_client.call_async(service_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def receive_signal_handler(self, data):
        print(data)
        # self.data_received_signal.emit(data)

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
        

class MyGUI(QWidget):
    def __init__(self, node):
        """_summary_

        Args:
            node (_type_): _description_
        """
        super().__init__()
        
        self.node = node
        self.layout_global = QVBoxLayout()

        self.init_recorder_ui()
        self.init_motor_ui()
        self.init_filter_checkbox()
        self.init_set_zero_button()
        self.init_fts_ui()
        self.init_loadcell_ui()
        self.init_fts_plot()
        self.init_timer()

    def init_recorder_ui(self):
        self.record_layout = QVBoxLayout()
        self.record_label = QLabel('Recording')
        self.record_button = QPushButton('Record')
        self.record_button.setStyleSheet('QPushButton {color: green;}')  # 초록색으로 변경
        self.record_button.clicked.connect(self.toggle_record)
        self.record_button.setFixedWidth(200)
        self.record_button.setFixedHeight(50)
        self.record_layout.addWidget(self.record_button)
        self.layout_global.addLayout(self.record_layout)

        self.is_recording = False
        pass

    def toggle_record(self):
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        self.record_button.setText('Stop')
        self.record_button.setStyleSheet('QPushButton {color: red;}')  # 빨간색으로 변경
        self.is_recording = True
        response = self.node.send_request_record_start()
        print(response.message)

    def stop_recording(self):
        self.record_button.setText('Record')
        self.record_button.setStyleSheet('QPushButton {color: green;}')  # 초록색으로 변경
        self.is_recording = False
        response = self.node.send_request_record_stop()
        print(response.message)

    def init_motor_ui(self):
        '''
        @ autor DY
        @ note motor subscriber and publisher list
        '''
        self.layout_mode = QVBoxLayout()
        self.label_mode = QLabel('Operation Mode')
        self.checkbox_mode_list = [QCheckBox('manual'), QCheckBox('kinematics'), QCheckBox('Dynamics')]
        self.checkbox_mode_list[0].setChecked(False)
        self.checkbox_mode_list[0].setFixedWidth(200)
        self.checkbox_mode_list[0].clicked.connect(self.checkbox_mode_clicked)
        self.checkbox_mode_list[0].stateChanged.connect(self.disable_mode)
        self.checkbox_mode_list[1].setChecked(True)
        self.checkbox_mode_list[1].setFixedWidth(200)
        self.checkbox_mode_list[1].clicked.connect(self.checkbox_mode_clicked)
        self.checkbox_mode_list[2].setChecked(False)
        self.checkbox_mode_list[2].setFixedWidth(200)
        self.checkbox_mode_list[2].clicked.connect(self.checkbox_mode_clicked)
        # self.checkbox_mode_list[1].stateChanged.connect(self.disable_mode)
        self.layout_mode.addWidget(self.label_mode)
        self.layout_mode.addWidget(self.checkbox_mode_list[0])
        self.layout_mode.addWidget(self.checkbox_mode_list[1])
        self.layout_mode.addWidget(self.checkbox_mode_list[2])
        self.layout_mode.setAlignment(self.label_mode, Qt.AlignRight)
        self.layout_mode.setAlignment(self.checkbox_mode_list[0], Qt.AlignRight)
        self.layout_mode.setAlignment(self.checkbox_mode_list[1], Qt.AlignRight)
        self.layout_mode.setAlignment(self.checkbox_mode_list[2], Qt.AlignRight)
        self.layout_global.addLayout(self.layout_mode)

        self.motor_layout_list = []
        self.motor_state_label_list = []
        self.motor_state_line_edit_list = []

        self.motor_pub_label_list = []
        self.motor_pub_line_edit_list = []
        self.motor_pub_button_list = []

        for i in range(self.node.numofmotors):  # make label, line_editor, push button for motor control
            self.motor_layout_list.append(QHBoxLayout())

            self.motor_state_label_list.append(QLabel(f'Motor #{i}  a_pos: '))
            self.motor_state_line_edit_list.append(QLineEdit('0'))

            self.motor_pub_label_list.append(QLabel('move(relative)'))
            self.motor_pub_line_edit_list.append(QLineEdit('0'))
            self.motor_pub_button_list.append(QPushButton('Publish'))

            # @ note  if you input 'i' into argument of function 'request_motor_move' directly,
            #         then all button will work about num=2, in this case
            try:
                if i == 0:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=0))
                elif i == 1:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=1))
                elif i == 2:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=2))
                elif i == 3:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=3))
                elif i == 4:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=4))
                elif i == 5:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=5))
                elif i == 6:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=6))
                elif i == 7:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=7))
                elif i == 8:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=8))
                elif i == 9:
                    self.motor_pub_button_list[i].clicked.connect(lambda : self.request_motor_move(num=8))
            except Exception as e:
                print(f'Exception error on connecting functions to button as {e}')
        


        # @autor DY
        # lambda F for apply the funtion to all the list arugments)
        list(map(lambda x: x.setAlignment(Qt.AlignVCenter | Qt.AlignRight), self.motor_state_label_list))        
        list(map(lambda x: x.setFixedWidth(150), self.motor_state_line_edit_list))
        list(map(lambda x: x.setFixedHeight(30), self.motor_state_line_edit_list))
        list(map(lambda x: x.setReadOnly(True), self.motor_state_line_edit_list))
        list(map(lambda x: x.setAlignment(Qt.AlignVCenter | Qt.AlignRight), self.motor_pub_label_list))
        list(map(lambda x: x.setFixedWidth(150), self.motor_pub_line_edit_list))
        list(map(lambda x: x.setFixedHeight(30), self.motor_pub_line_edit_list))
        list(map(lambda x: x.setFixedWidth(150), self.motor_pub_button_list))
        list(map(lambda x: x.setFixedHeight(30), self.motor_pub_button_list))        


        for i in range(self.node.numofmotors):
            try:
                self.motor_layout_list[i].addWidget(self.motor_state_label_list[i])
                self.motor_layout_list[i].addWidget(self.motor_state_line_edit_list[i])
                self.motor_layout_list[i].addWidget(self.motor_pub_label_list[i])
                self.motor_layout_list[i].addWidget(self.motor_pub_line_edit_list[i])
                self.motor_layout_list[i].addWidget(self.motor_pub_button_list[i])
                self.motor_pub_button_list[i].setEnabled(False)
                self.layout_global.addLayout(self.motor_layout_list[i])
            finally:
                pass
        

        self.layout_amode = QVBoxLayout()
        self.label_amode = QLabel('Actuation mode')
        self.checkbox_amode_list = [QCheckBox('Absolute'), QCheckBox('Relative')]
        self.checkbox_amode_list[0].setChecked(False)
        self.checkbox_amode_list[0].setFixedWidth(200)
        self.checkbox_amode_list[0].clicked.connect(self.checkbox_amode_clicked)
        self.checkbox_amode_list[1].setChecked(True)
        self.checkbox_amode_list[1].setFixedWidth(200)
        self.checkbox_amode_list[1].clicked.connect(self.checkbox_amode_clicked)
        self.layout_amode.addWidget(self.label_amode)
        self.layout_amode.addWidget(self.checkbox_amode_list[0])
        self.layout_amode.addWidget(self.checkbox_amode_list[1])
        self.layout_amode.setAlignment(self.label_amode, Qt.AlignRight)
        self.layout_amode.setAlignment(self.checkbox_amode_list[0], Qt.AlignRight)
        self.layout_amode.setAlignment(self.checkbox_amode_list[1], Qt.AlignRight)
        self.layout_global.addLayout(self.layout_amode)
        
        self.motor_kinematics_layout = QVBoxLayout()
        self.motor_kinematics_label_list = []
        self.motor_kinematics_label_list.append(QLabel("Move Tip(Degree) | Pan (S-N)"))
        self.motor_kinematics_label_list.append(QLabel("Move Tip(Degree) | Tilt(E-W)"))

        self.motor_kinematics_line_edit_list = []
        self.motor_kinematics_layout_list = []
        for i in range (len(self.motor_kinematics_label_list)):  # 6 (force 3d, torque 3d)
            try:
                self.motor_kinematics_layout_list.append(QHBoxLayout())
                self.motor_kinematics_line_edit_list.append(QLineEdit('0'))
                self.motor_kinematics_layout_list[i].addWidget(self.motor_kinematics_label_list[i])
                self.motor_kinematics_layout_list[i].addWidget(self.motor_kinematics_line_edit_list[i])
                self.motor_kinematics_layout.addLayout(self.motor_kinematics_layout_list[i])
            finally:
                pass
        list(map(lambda x: x.setAlignment(Qt.AlignVCenter | Qt.AlignRight), self.motor_kinematics_label_list))        
        list(map(lambda x: x.setFixedWidth(100), self.motor_kinematics_line_edit_list))
        list(map(lambda x: x.setFixedHeight(30), self.motor_kinematics_line_edit_list))
        self.motor_kinematics_button = QPushButton('Publish')
        self.motor_kinematics_button.clicked.connect(self.request_motor_move)
        self.motor_kinematics_button.setFixedWidth(150)
        self.motor_kinematics_button.setFixedHeight(70)

        self.motor_kinematics_layout_fin = QHBoxLayout()
        self.motor_kinematics_layout_fin.addLayout(self.motor_kinematics_layout)
        self.motor_kinematics_layout_fin.addWidget(self.motor_kinematics_button)
        self.layout_global.addLayout(self.motor_kinematics_layout_fin)

        # self.motor_kinematics_label = QLabel("Move Tip(Degree) | Tilt")
        # self.motor_kinematics_line_edit = QLineEdit('0')
        # self.motor_kinematics_button = QPushButton('Publish')
        # self.motor_kinematics_button.clicked.connect(self.request_motor_move)
        # self.motor_kinematics_label.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        # self.motor_kinematics_button.setFixedWidth(150)
        # self.motor_kinematics_button.setFixedHeight(30)
        # self.motor_kinematics_line_edit.setFixedWidth(100)
        # self.motor_kinematics_line_edit.setFixedHeight(30)

        # self.motor_kinematics_layout = QHBoxLayout()
        # self.motor_kinematics_layout.addWidget(self.motor_kinematics_label)
        # self.motor_kinematics_layout.addWidget(self.motor_kinematics_line_edit)
        # self.motor_kinematics_layout.addWidget(self.motor_kinematics_button)
        # self.layout_global.addLayout(self.motor_kinematics_layout)

    def init_filter_checkbox(self):
        self.layout_fileter_checkbox = QVBoxLayout()

        self.layout_LPF = QHBoxLayout()
        self.layout_MAF = QHBoxLayout()

        self.LPF_parameter_label = QLabel('Weight (0 ~ 1):')
        self.LPF_parameter_label.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.LPF_parameter = QLineEdit('0.5')
        self.LPF_parameter.setFixedWidth(100)
        self.LPF_parameter.setFixedHeight(30)

        self.MAF_parameter_label = QLabel('Buffer size:')
        self.MAF_parameter_label.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.MAF_parameter = QLineEdit('5')
        self.MAF_parameter.setFixedWidth(100)
        self.MAF_parameter.setFixedHeight(30)

        self.layout_LPF.addWidget(self.LPF_parameter_label)
        self.layout_LPF.addWidget(self.LPF_parameter)
        self.layout_MAF.addWidget(self.MAF_parameter_label)
        self.layout_MAF.addWidget(self.MAF_parameter)
        
        self.checkbox_filter_list = [QCheckBox('Weight filter(LPF)'), QCheckBox('Moving avg filter(MAF)')]
        self.checkbox_filter_list[0].setChecked(False)
        self.checkbox_filter_list[0].setFixedWidth(350)
        self.checkbox_filter_list[1].setChecked(False)
        self.checkbox_filter_list[1].setFixedWidth(350)
        self.layout_LPF.addWidget(self.checkbox_filter_list[0])
        self.layout_MAF.addWidget(self.checkbox_filter_list[1])
        self.layout_LPF.setAlignment(self.checkbox_filter_list[0], Qt.AlignRight)
        self.layout_MAF.setAlignment(self.checkbox_filter_list[1], Qt.AlignRight)
        self.layout_global.addLayout(self.layout_LPF)
        self.layout_global.addLayout(self.layout_MAF)

        # self.checkbox_amode_list[0].clicked.connect(self.checkbox_amode_clicked)
       
        # self.filter_checkbox = []

    def init_set_zero_button(self):
        self.zero_button = QPushButton('Set zero')
        self.zero_button.clicked.connect(self.request_set_zero)
        self.zero_button.setFixedWidth(150)
        self.zero_button.setFixedHeight(70)
        self.zero_button_layout = QVBoxLayout()
        self.zero_button_layout.addWidget(self.zero_button)
        self.layout_global.addLayout(self.zero_button_layout)

    def init_fts_ui(self):
        '''
        @ autor DY
        @ note force-torque sensor list for monitoring
        '''
        
        self.fts_sub_label_list = []
        self.fts_sub_line_edit_list = []
        self.fts_sub_layout_list = []
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_fx'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_fy'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_fz'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_tx'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_ty'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_tz'))

        for i in range (len(self.fts_sub_label_list)):  # 6 (force 3d, torque 3d)
            try:
                self.fts_sub_layout_list.append(QHBoxLayout())
                self.fts_sub_line_edit_list.append(QLineEdit('0'))
                self.fts_sub_layout_list[i].addWidget(self.fts_sub_label_list[i])
                self.fts_sub_layout_list[i].addWidget(self.fts_sub_line_edit_list[i])
                self.layout_global.addLayout(self.fts_sub_layout_list[i])
            finally:
                pass

        list(map(lambda x: x.setReadOnly(True), self.fts_sub_line_edit_list))

    def init_loadcell_ui(self):
        '''
        @ autor DY
        @ note force-torque sensor list for monitoring
        '''
        self.lc_sub_label_list = []
        self.lc_sub_line_edit_list = []
        self.lc_sub_layout_list = []
        self.lc_sub_label_list.append(QLabel('loadcell #1 weight(kg)'))
        self.lc_sub_label_list.append(QLabel('loadcell #2 weight(kg)'))

        for i in range (len(self.lc_sub_label_list)):  # 6 (force 3d, torque 3d)
            try:
                self.lc_sub_layout_list.append(QHBoxLayout())
                self.lc_sub_line_edit_list.append(QLineEdit('NAN'))
                self.lc_sub_layout_list[i].addWidget(self.lc_sub_label_list[i])
                self.lc_sub_layout_list[i].addWidget(self.lc_sub_line_edit_list[i])
                self.layout_global.addLayout(self.lc_sub_layout_list[i])
            finally:
                pass

        list(map(lambda x: x.setReadOnly(True), self.lc_sub_line_edit_list))


    def init_timer(self):
        '''
        @ author DY
        @ note Timer objects
        '''
        try:
            self.setLayout(self.layout_global)

            # motor_state update
            self.timer_motor_state = QTimer(self)
            self.timer_motor_state.timeout.connect(self.update_motor_state)
            self.timer_motor_state.start(33)

            # fts_data update
            self.timer_fts = QTimer(self)
            self.timer_fts.timeout.connect(self.update_fts)
            self.timer_fts.start(33)

            # fts_data update
            self.timer_loadcell = QTimer(self)
            self.timer_loadcell.timeout.connect(self.update_loadcell)
            self.timer_loadcell.start(33)

            # filter_checkbox_state update
            self.timer_loadcell = QTimer(self)
            self.timer_loadcell.timeout.connect(self.update_filter_state)
            self.timer_loadcell.start(500)

            # ros node
            self.timer_ros_node = QTimer(self)
            self.timer_ros_node.timeout.connect(self.node_spin_once)
            self.timer_ros_node.start(2)  # 10 밀리초 주기로 타이머 실행
        except Exception as e:
            self.node.get_logger().warning(f'F:init_timer() -> {e}')


    def init_fts_plot(self):
        try:
            # Matplotlib graph
            self.figure, self.ax = plt.subplots()
            self.canvas = FigureCanvas(self.figure)
            self.layout_global.addWidget(self.canvas)
            self.data_y = np.zeros((6, 50))  # 초기 데이터 설정 (6개의 데이터, 각각 100개의 요소)
            self.data_x = [i for i in range(len(self.data_y))]

            # 그래프 초기화
            self.lines = [self.ax.plot([], [], label=f'Data {i}')[0] for i in range(6)]
            self.ax.legend()
            # 애니메이션 시작
            self.animation = FuncAnimation(self.figure, self.update_fts_plot, frames=100, interval=25)
            # self.node.data_received_signal.connect(self.generateTimerRosNode)

        except Exception as e:
            self.node.get_logger().warning(f'F:FuncAnimation() -> {e}')
            
    
    # def generateTimerRosNode(self):
    #     try:
    #         self.timer_ros_node = QTimer(self)
    #         self.timer_ros_node.timeout.connect(self.node_spin_once)
    #         self.timer_ros_node.start(10)  # 10 밀리초 주기로 타이머 실행
    #     except Exception as e:
    #         self.node.get_logger().warning(f'F:FuncAnimation() -> {e}')
    def checkbox_mode_clicked(self):
        sender = self.sender()

        # manual
        if sender == self.checkbox_mode_list[0] and self.checkbox_mode_list[0].isChecked():
            self.node.send_request_change_control_mode(mode=False)
            self.checkbox_mode_list[1].setChecked(False)
            self.checkbox_mode_list[2].setChecked(False)
        # kinematics
        elif sender == self.checkbox_mode_list[1] and self.checkbox_mode_list[1].isChecked():
            self.node.get_logger().info("Setting parameter...")
            self.node.send_request_change_control_mode(mode=False)
            self.checkbox_mode_list[0].setChecked(False)
            self.checkbox_mode_list[2].setChecked(False)

        # dynamics
        elif sender == self.checkbox_mode_list[2] and self.checkbox_mode_list[2].isChecked():
            self.node.get_logger().info("Setting parameter...")
            self.node.send_request_change_control_mode(mode=True)
            self.checkbox_mode_list[0].setChecked(False)
            self.checkbox_mode_list[1].setChecked(False)

    def checkbox_amode_clicked(self):
        sender = self.sender()
        if sender == self.checkbox_amode_list[0] and self.checkbox_amode_list[0].isChecked():
            self.checkbox_amode_list[1].setChecked(False)
        elif sender == self.checkbox_amode_list[1] and self.checkbox_amode_list[1].isChecked():
            self.checkbox_amode_list[0].setChecked(False)        
    

    def disable_mode(self, state):
        if state == Qt.Checked:
            list(map(lambda x: x.setEnabled(True), self.motor_pub_button_list))
            self.motor_kinematics_button.setEnabled(False)
        else:
            list(map(lambda x: x.setEnabled(False), self.motor_pub_button_list))
            self.motor_kinematics_button.setEnabled(True)
        pass

    def create_subscriber_gui(self,
                              label_text='label',
                              *,
                              line_edit_text='0'
                              ):
        label = QLabel(f"{label_text}")
        line_edit = QLineEdit(f"{line_edit_text}")
        pass

    def request_motor_move(self, num=0):
        try:
            if len(self.node.motor_state.actual_position) ==0:
                # self.node.get_logger().warning(f'command val : {cmd_val}')
                self.node.get_logger().warning(f'motor_state does not exit. Check the connection')
                return
            else:

                # Mode : Manual operation of each motors
                if self.checkbox_mode_list[0].isChecked():
                    if self.node.opmode == '0x08':   # CSP
                        # cmd_val = self.node.motor_state.actual_position[num] + (int(self.motor_pub_line_edit_list[num].text()))
                        cmd_val = int(self.motor_pub_line_edit_list[num].text())
                        print('python', cmd_val)
                        response = self.node.send_request_move_motor_direct(idx=num, tp=cmd_val, tvp=50)
                    elif self.node.opmode == '0x09': # CSV
                        cmd_val = int(self.motor_pub_line_edit_list[num].text())
                    self.node.get_logger().info(f'motor #{num} -> {cmd_val} command update {response}.')

                # Mode : Operated by the inverse-kinematics of manipulator
                elif self.checkbox_mode_list[1].isChecked():
                    # Calculate the encoder value from the degree of surgical tool target

                    if self.checkbox_amode_list[0].isChecked(): # Absolute
                        response = self.node.send_request_move_tool_angle(pan=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                          tilt=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                          mode=0)
                    elif self.checkbox_amode_list[1].isChecked():
                        response = self.node.send_request_move_tool_angle(pan=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                          tilt=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                          mode=1)
                # Mode : Dynamics
                elif self.checkbox_mode_list[2].isChecked():
                    response = self.node.send_request_move_tool_angle_dynamics(pan=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                      tilt=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                      mode=0)
                    pass
        except Exception as e:
            self.node.get_logger().warning(f'F:request_motor_move() -> {e}')
            return

    def request_set_zero(self):
        try:
            response = self.node.send_request_set_zero()
        except Exception as e:
            self.node.get_logger().warning(f'F:set_zero() -> {e}')

    def node_spin_once(self):
        rclpy.spin_once(self.node)


    def update_motor_state(self):
        try:
            for i in range(self.node.numofmotors):
                self.motor_state_line_edit_list[i].setText(str(self.node.motor_state.actual_position[i]))
        except Exception as e:
            self.node.get_logger().warning(f'F:update_motor_state() -> {e}')
        return

    def update_fts(self):
        try:
            self.fts_sub_line_edit_list[0].setText(str(self.node.fts_data.wrench.force.x))
            self.fts_sub_line_edit_list[1].setText(str(self.node.fts_data.wrench.force.y))
            self.fts_sub_line_edit_list[2].setText(str(self.node.fts_data.wrench.force.z))
            self.fts_sub_line_edit_list[3].setText(str(self.node.fts_data.wrench.torque.x))
            self.fts_sub_line_edit_list[4].setText(str(self.node.fts_data.wrench.torque.y))
            self.fts_sub_line_edit_list[5].setText(str(self.node.fts_data.wrench.torque.z))
        except Exception as e:
            # self.node.get_logger().warning(f'F:update_fts() -> {e}')
            pass
        return 1
    
    def update_loadcell(self):
        try:
            self.lc_sub_line_edit_list[0].setText(str(self.node.loadcell_data.stress[0]))
            self.lc_sub_line_edit_list[1].setText(str(self.node.loadcell_data.stress[1]))
        except Exception as e:
            # self.node.get_logger().warning(f'F:update_loadcell() -> {e}')
            pass
        finally:
            return 1
    
    def update_fts_plot(self, frame):
        try:
            new_data = np.zeros(6)
            if rclpy.ok():
                new_data[0] = (self.node.fts_data.wrench.force.x)
                new_data[1] = (self.node.fts_data.wrench.force.y)
                new_data[2] = (self.node.fts_data.wrench.force.z)
                new_data[3] = (self.node.fts_data.wrench.torque.x)
                new_data[4] = (self.node.fts_data.wrench.torque.y)
                new_data[5] = (self.node.fts_data.wrench.torque.z)

                self.data_y = np.roll(self.data_y, shift=-1, axis=1)
                self.data_y[:, -1] = new_data

                for i, line in enumerate(self.lines):
                    line.set_data(range(len(self.data_y[i])), self.data_y[i])

                # 최댓값과 최솟값을 찾아 축의 범위 설정
                min_val = np.min(self.data_y)
                max_val = np.max(self.data_y)
                margin = 1000  # 여분의 여백 설정
                self.ax.set_xlim(0, len(self.data_y[0]) - 1)
                self.ax.set_ylim(min_val - margin, max_val + margin)

                return self.lines
        
        except Exception as e:
            self.node.get_logger().warning(f'F:update_fts_plot() -> {e}')
            return self.lines

    def update_filter_state(self):
        msg = DataFilterSetting()
        msg.set_lpf = self.checkbox_filter_list[0].isChecked()
        msg.set_maf = self.checkbox_filter_list[1].isChecked()
        msg.lpf_weight = float(self.LPF_parameter.text())
        msg.maf_buffer_size = int(self.MAF_parameter.text())

        self.node.data_filter_setting_publisher.publish(msg)


def main():

    rclpy.init(args=None)
    node = GUINode()
    app = QApplication(sys.argv)
    gui = MyGUI(node)
    gui.show()

    sys.exit(app.exec_())
    # rclpy.init(args=None)
    # node = GUINode()

    # # 멀티스레드 실행기를 사용하여 ROS2 노드를 별도의 스레드에서 실행
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)

    # # 별도 스레드에서 spin 시작
    # spin_thread = threading.Thread(target=executor.spin, daemon=True)
    # spin_thread.start()

    # # PyQt5 GUI 실행
    # app = QApplication(sys.argv)
    # gui = MyGUI(node)
    # gui.show()

    # # GUI 종료 시 실행기와 스레드도 정리
    # exit_code = app.exec_()
    # executor.shutdown()
    # node.destroy_node()
    # rclpy.shutdown()
    # spin_thread.join()
    # sys.exit(exit_code)

if __name__ == '__main__':
    main()

