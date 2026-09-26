import json
from pathlib import Path
import signal
import sys
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QDialog
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QScrollArea
from PyQt5.QtWidgets import QSplitter
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation
import numpy as np

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
# from rclpy import RCLError
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool
from custom_interfaces.msg import LoadcellState
from custom_interfaces.msg import MotorCommand
from custom_interfaces.msg import MotorState
from custom_interfaces.msg import DataFilterSetting
from custom_interfaces.msg import AdmittanceControl
from custom_interfaces.msg import PositionControl
from custom_interfaces.srv import MoveMotorDirect
from custom_interfaces.srv import MoveToolAngle
from custom_interfaces.srv import SetGoalPosition
from custom_interfaces.srv import SetControlMode


from enum import Enum
from ament_index_python.packages import get_package_share_directory
from gui_py_pkg.system_manager import SystemProcessManager
from gui_py_pkg.system_manager import load_component_specs
from gui_py_pkg.image_preview import ImagePreviewPanel
from gui_py_pkg.serial_port_selector import SerialPortSelector
from gui_py_pkg.camera_selector import CameraSelector
from gui_py_pkg.camera_exposure import CameraExposurePanel
from gui_py_pkg.sine_motion import SineMotionPanel
from gui_py_pkg.force_alignment import ForceAlignmentPanel, request_record_start
from gui_py_pkg.roi_editor import RoiEditorDialog, load_roi_config
# 제어모드를 나타내는 Enum 정의
class ControlMode(Enum):
    kKinematics = 1
    kDynamics = 2
    kPosition = 3
    kAdmittance = 4

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')

        # Keep the GUI installable and independent of a source-tree-relative
        # C++ header. These parameters mirror hw_definition.hpp and can be
        # overridden from the GUI launch file when the hardware changes.
        self.declare_parameter('num_motors', 4)
        self.declare_parameter('num_loadcells', 4)
        self.declare_parameter('op_mode', 8)
        self.declare_parameter('auto_start_components', True)
        self.numofmotors = int(self.get_parameter('num_motors').value)
        self.num_loadcells = int(self.get_parameter('num_loadcells').value)
        if self.num_loadcells < 1:
            raise ValueError('num_loadcells must be at least 1')
        self.opmode = int(self.get_parameter('op_mode').value)
        self.auto_start_components = bool(
            self.get_parameter('auto_start_components').value
        )
        self.get_logger().info(
            f'Configured {self.numofmotors} motors; '
            f'operation mode=0x{self.opmode:02x}'
        )

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        self.record_status = None
        self.record_status_subscriber = self.create_subscription(
            String, '/data/record_status', self.receive_record_status,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

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
        self.last_fts_data_time = None
        self.fts_data = WrenchStamped()
        self.fts_subscriber = self.create_subscription(
            WrenchStamped,
            'fts_data',
            self.read_fts_data,
            QOS_RKL10V
        )
        self.get_logger().info('fts_data subscriber is created.')

        self.loadcell_data = LoadcellState()
        self.last_loadcell_data_time = None
        self.loadcell_subscriber = self.create_subscription(
            LoadcellState,
            'loadcell_state',
            self.read_loadcell_data,
            QOS_RKL10V
        )
        self.get_logger().info('loadcell_data subscriber is created.')

        self.motor_state = MotorState()
        self.last_motor_state_time = None
        self.motor_state_subscriber = self.create_subscription(
            MotorState,
            'motor_state',
            self.read_motor_state,
            QOS_RKL10V
        )
        self.get_logger().info('motor_state subscriber is created.')

        # IK preview order is fixed by robot_control_pkg:
        # [East, West, South, North], in millimetres.
        self.target_wire_length = [0.0] * self.numofmotors
        self.target_wire_length_subscriber = self.create_subscription(
            Float64MultiArray,
            'kinematics/target_wire_length',
            self.read_target_wire_length,
            QOS_RKL10V
        )
        self.get_logger().info('kinematics/target_wire_length subscriber is created.')
        
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
            'move_motor_direct'
        )

        self.move_tool_angle_service_client = self.create_client(
            MoveToolAngle,
            'kinematics/move_tool_angle'
        )

        self.move_tool_angle_dynamics_service_client = self.create_client(
            MoveToolAngle,
            'dynamics/move_tool_angle'
        )

        self.set_goal_position_service_client = self.create_client(
            SetGoalPosition,
            'position/set_goal_position'
        )

        self.set_control_mode_service_client = self.create_client(
            SetControlMode,
            'control/set_control_mode'
        )

        self.robot_control_parameter_client = self.create_client(
            SetParameters,
            '/robot_control/set_parameters',
        )

        # self.control_mode_kinematics_and_dynamics_client = self.create_client(
        #     SetBool,
        #     'control/control_mode_kin_dyn'
        # )
        # while not self.control_mode_kinematics_and_dynamics_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().warning('The "control/control_mode_kin_dyn" service server not available. Check the kinematics_control_node')

        # self.control_mode_position_and_admittance_client = self.create_client(
        #     SetBool,
        #     'control/control_mode_pos_admit'
        # )
        # while not self.control_mode_position_and_admittance_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().warning('The "control/control_mode_pos_admit" service server not available. Check the kinematics_control_node')

        self.recoder_service_client = self.create_client(
            SetBool,
            '/data/record'
        )
        self.record_parameter_client = self.create_client(
            SetParametersAtomically, '/record/set_parameters_atomically')

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
        self.last_fts_data_time = time.monotonic()
        # self.data_received_signal.emit(True)     # DY
        self.fts_data = msg

    def read_loadcell_data(self, msg):
        self.last_loadcell_data_time = time.monotonic()
        self.loadcell_data = msg

    def read_motor_state(self, msg):
        self.last_motor_state_time = time.monotonic()
        self.motor_state = msg
        # pass

    def read_target_wire_length(self, msg):
        if len(msg.data) != self.numofmotors:
            self.get_logger().warning(
                'Expected %d IK wire targets, received %d.',
                self.numofmotors,
                len(msg.data)
            )
            return
        self.target_wire_length = list(msg.data)

    def discovered_node_names(self):
        """Return fully qualified names currently visible in the ROS graph."""

        discovered = set()
        for name, namespace in self.get_node_names_and_namespaces():
            if namespace == '/':
                discovered.add(f'/{name}')
            else:
                discovered.add(f'{namespace.rstrip("/")}/{name}')
        return discovered

    def set_motor_output_enabled(self, enabled):
        """Set the robot controller safety gate without blocking the GUI."""

        if not self.robot_control_parameter_client.service_is_ready():
            return None
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'motor_output_enabled',
                Parameter.Type.BOOL,
                bool(enabled),
            ).to_parameter_msg()
        ]
        return self.robot_control_parameter_client.call_async(request)

    def publish(self, msg):
        self.motor_command_publisher_.publish(msg)

    def call_service_async(self, client, request, service_name):
        if not client.service_is_ready():
            self.get_logger().warning(
                f'Service is not available: {service_name}'
            )
            return None
        return client.call_async(request)

    def send_request_move_motor_direct(self, idx=0, tp=0, tvp=50):
        service_request = MoveMotorDirect.Request()
        service_request.index_motor = idx
        service_request.target_position = tp
        service_request.target_velocity_profile = tvp
        return self.call_service_async(
            self.move_motor_direct_service_client,
            service_request,
            '/move_motor_direct',
        )
    
    def send_request_move_tool_angle(self, pan=0.0, tilt=0.0, grip=0.0, mode=0):
        service_request = MoveToolAngle.Request()
        service_request.panangle = pan
        service_request.tiltangle = tilt
        service_request.gripangle = grip
        service_request.mode = mode
        return self.call_service_async(
            self.move_tool_angle_service_client,
            service_request,
            '/kinematics/move_tool_angle',
        )
    
    def send_request_move_tool_angle_dynamics(self, pan=0.0, tilt=0.0, grip=0.0, mode=0):
        service_request = MoveToolAngle.Request()
        service_request.panangle = pan
        service_request.tiltangle = tilt
        service_request.gripangle = grip
        service_request.mode = mode
        return self.call_service_async(
            self.move_tool_angle_dynamics_service_client,
            service_request,
            '/dynamics/move_tool_angle',
        )
    
    def send_request_set_goal_position(self, x=0.0, y=0.0, z=0.0, ref='absolute'):
        service_request = SetGoalPosition.Request()
        service_request.reference_type = ref
        service_request.goal_position.position.x = x
        service_request.goal_position.position.y = y
        service_request.goal_position.position.z = z
        service_request.goal_position.orientation.x = 0.0
        service_request.goal_position.orientation.y = 0.0
        service_request.goal_position.orientation.z = 0.0
        service_request.goal_position.orientation.w = 0.0
        return self.call_service_async(
            self.set_goal_position_service_client,
            service_request,
            '/position/set_goal_position',
        )
    
    # mode : true-dynamics / false-kinematics
    def send_request_change_control_mode_kinematics_and_dynamics(self, mode=False):
        service_request = SetBool.Request()
        service_request.data = mode
        future = self.control_mode_kinematics_and_dynamics_client.call_async(service_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_request_set_control_mode(self, mode:Enum=ControlMode.kKinematics): 
        """Set control mode of HRM

        Args:
            mode (int, optional): followd "ControlNode" class. Defaults to 1.
            kinematics=1,
            dynamics=2,
            position=3,
            admittance=4
        Returns:
            service response (ROS) : result of success and message (ROS)
        """
        service_request = SetControlMode.Request()
        service_request.mode = mode.value
        return self.call_service_async(
            self.set_control_mode_service_client,
            service_request,
            '/control/set_control_mode',
        )
    
    def receive_record_status(self, message):
        try:
            self.record_status = (json.loads(message.data), time.monotonic())
        except (TypeError, ValueError):
            self.get_logger().warning('Invalid recorder status message')

    def send_request_record_start(self, alignment, contact_segment_id=0):
        return request_record_start(
            self.record_parameter_client, self.recoder_service_client, alignment,
            contact_segment_id)
    
    def send_request_record_stop(self):
        service_request = SetBool.Request()
        service_request.data = False
        return self.call_service_async(
            self.recoder_service_client,
            service_request,
            '/data/record',
        )

    def send_request_set_zero(self):
        service_request = SetBool.Request()
        service_request.data = True
        return self.call_service_async(
            self.set_zero_client,
            service_request,
            '/serial_data/set_zero',
        )

    def receive_signal_handler(self, data):
        print(data)
        # self.data_received_signal.emit(data)

class MyGUI(QWidget):
    def __init__(self, node):
        """_summary_

        Args:
            node (_type_): _description_
        """
        super().__init__()

        self.node = node
        self.is_closing = False
        # A conventional desktop control font leaves room for all three live
        # previews without forcing the motor/record controls off-screen.
        control_font = self.font()
        control_font.setPointSizeF(min(control_font.pointSizeF(), 10.0))
        self.setFont(control_font)
        self.setWindowTitle('Hyper-Redundant Manipulator Control')
        self.resize(1500, 900)

        root_layout = QHBoxLayout(self)
        root_layout.setContentsMargins(6, 6, 6, 6)
        self.main_splitter = QSplitter(Qt.Horizontal)
        self.left_splitter = QSplitter(Qt.Vertical)
        root_layout.addWidget(self.main_splitter)

        self.system_tab = QWidget()
        self.control_tab = QWidget()
        self.sensor_tab = QWidget()
        self.sensor_raw_widget = QWidget()
        self.sensor_plot_widget = QWidget()
        self.system_layout = QVBoxLayout(self.system_tab)
        self.control_layout = QVBoxLayout(self.control_tab)
        self.control_layout.setContentsMargins(6, 4, 6, 4)
        self.control_layout.setSpacing(2)
        self.sensor_layout = QHBoxLayout(self.sensor_tab)
        self.sensor_raw_layout = QVBoxLayout(self.sensor_raw_widget)
        self.sensor_plot_layout = QVBoxLayout(self.sensor_plot_widget)

        self.sensor_raw_scroll = QScrollArea()
        self.sensor_raw_scroll.setWidgetResizable(True)
        self.sensor_raw_scroll.setWidget(self.sensor_raw_widget)

        self.sensor_splitter = QSplitter(Qt.Horizontal)
        self.sensor_splitter.addWidget(self.sensor_raw_scroll)
        self.sensor_splitter.addWidget(self.sensor_plot_widget)
        self.sensor_splitter.setStretchFactor(0, 1)
        self.sensor_splitter.setStretchFactor(1, 2)
        self.sensor_splitter.setSizes([300, 520])
        self.sensor_layout.addWidget(self.sensor_splitter)

        self.left_splitter.addWidget(self.system_tab)
        self.left_splitter.addWidget(self.sensor_tab)
        self.left_splitter.setStretchFactor(0, 1)
        self.left_splitter.setStretchFactor(1, 1)
        self.left_splitter.setSizes([430, 470])

        self.main_splitter.addWidget(self.left_splitter)
        # A normal layout can shrink the image canvases to their useful minimum;
        # QScrollArea instead reserves their larger height-for-width size hint.
        self.main_splitter.addWidget(self.control_tab)
        self.main_splitter.setStretchFactor(0, 2)
        self.main_splitter.setStretchFactor(1, 3)
        self.main_splitter.setSizes([760, 1160])

        self.init_system_ui()
        self.layout_global = self.control_layout
        self.add_section_heading('Motor control')
        self.init_motor_safety_ui()
        self.init_motor_ui()
        self.init_filter_checkbox()
        self.init_recorder_ui()
        self.image_preview = ImagePreviewPanel(self.node.context)
        self.control_layout.addWidget(self.image_preview, 1)

        self.layout_global = self.sensor_raw_layout
        self.add_section_heading('Raw sensor data')
        self.init_fts_ui()
        self.init_loadcell_ui()
        self.init_set_zero_button()
        self.sensor_raw_layout.addStretch(1)

        self.layout_global = self.sensor_plot_layout
        self.add_section_heading('Sensor monitoring')
        self.init_fts_plot()
        self.init_timer()

        if self.node.auto_start_components:
            self.schedule_auto_start()

    def add_section_heading(self, text):
        heading = QLabel(text)
        heading.setStyleSheet('font-size: 18px; font-weight: 600;')
        self.layout_global.addWidget(heading)

    def _system_config_path(self):
        try:
            share = Path(get_package_share_directory('gui_py_pkg'))
            installed = share / 'config/system_components.json'
            if installed.is_file():
                return installed
        except Exception:
            pass
        return Path(__file__).resolve().parents[1] / 'config/system_components.json'

    def init_system_ui(self):
        specs = load_component_specs(self._system_config_path())
        self.system_manager = SystemProcessManager(specs)
        self.component_widgets = {}
        self.auto_start_timers = []
        self.motor_gate_initialized_for_node = False
        self.auto_start_enabled = {
            spec.component_id: spec.auto_start for spec in specs
        }

        heading = QLabel('ROS 2 system components')
        heading.setStyleSheet('font-size: 18px; font-weight: 600;')
        self.system_layout.addWidget(heading)

        explanation = QLabel(
            'Green means the expected ROS node is visible. Processes started '
            'outside this GUI are shown in blue and are never stopped by it.'
        )
        explanation.setWordWrap(True)
        self.system_layout.addWidget(explanation)

        self.serial_port_selector = SerialPortSelector()
        self.system_layout.addWidget(self.serial_port_selector)

        self.roi_editor_active = False
        self.roi_dialog = None
        self.estimation_roi = None
        self.roi_config_path = None
        try:
            self.roi_config_path = (
                Path(get_package_share_directory('estimation_pkg')) / 'config_ROI_ref.json')
            self.estimation_roi = load_roi_config(self.roi_config_path)
        except (LookupError, OSError, ValueError) as error:
            self.node.get_logger().warning(f'Cannot load estimation ROI: {error}')
        roi_row = QHBoxLayout()
        self.roi_label = QLabel()
        self.roi_label.setWordWrap(True)
        self.roi_button = QPushButton('Estimation ROI…')
        self.roi_button.clicked.connect(self.edit_estimation_roi)
        roi_row.addWidget(self.roi_label, 1)
        roi_row.addWidget(self.roi_button)
        self.system_layout.addLayout(roi_row)
        self.update_roi_label()

        components_box = QGroupBox('Packages')
        components_layout = QVBoxLayout(components_box)
        self.camera_selector = CameraSelector()
        self.tag_camera_selector = CameraSelector(
            default_model='D455', substitution_prefix='tag_camera')
        self.camera_selectors = {
            'camera': self.camera_selector,
            'tag_camera': self.tag_camera_selector,
        }
        for spec in specs:
            row = QHBoxLayout()
            status_label = QLabel('● STOPPED')
            status_label.setMinimumWidth(105)
            name_label = QLabel(spec.label)
            name_label.setMinimumWidth(150)
            description_label = QLabel(spec.description)
            description_label.setStyleSheet('color: #666;')
            description_label.setWordWrap(True)
            auto_checkbox = QCheckBox(
                f'Auto ({spec.delay_sec:g}s)'
            )
            auto_checkbox.setChecked(spec.auto_start)
            auto_checkbox.toggled.connect(
                lambda checked, component_id=spec.component_id:
                self.auto_start_enabled.__setitem__(component_id, checked)
            )
            action_button = QPushButton('Start')
            action_button.setMinimumWidth(70)
            action_button.clicked.connect(
                lambda checked=False, component_id=spec.component_id:
                self.toggle_component(component_id)
            )

            row.addWidget(status_label)
            details = QVBoxLayout()
            details.setSpacing(0)
            details.addWidget(name_label)
            if spec.component_id in self.camera_selectors:
                # Reuse the existing description line, not another panel/row.
                details.addWidget(self.camera_selectors[spec.component_id])
                name_label.setToolTip(spec.description)
            else:
                details.addWidget(description_label)
            row.addLayout(details, 1)
            row.addWidget(auto_checkbox)
            row.addWidget(action_button)
            components_layout.addLayout(row)
            self.component_widgets[spec.component_id] = {
                'status': status_label,
                'button': action_button,
                'auto': auto_checkbox,
            }
        self.camera_exposure = CameraExposurePanel(self.node)
        components_layout.addWidget(self.camera_exposure)
        self.system_layout.addWidget(components_box)

        actions = QHBoxLayout()
        start_all_button = QPushButton('Start auto components')
        stop_all_button = QPushButton('Stop GUI-managed components')
        start_all_button.clicked.connect(self.schedule_auto_start)
        stop_all_button.clicked.connect(self.stop_all_components)
        actions.addWidget(start_all_button)
        actions.addWidget(stop_all_button)
        actions.addStretch(1)
        self.system_layout.addLayout(actions)
        self.system_layout.addStretch(1)

    def init_motor_safety_ui(self):
        self.motor_safety_box = QGroupBox()
        self.motor_safety_box.setToolTip(
            'Actuator safety: commands stay preview-only until physical motor output is enabled.')
        safety_layout = QHBoxLayout(self.motor_safety_box)
        safety_layout.setContentsMargins(8, 6, 8, 6)
        self.motor_output_checkbox = QCheckBox('Enable physical motor output')
        self.motor_output_checkbox.setChecked(False)
        self.motor_output_checkbox.toggled.connect(self.motor_output_toggled)
        self.motor_output_note = QLabel(
            'OFF — IK preview only; motor_command is blocked'
        )
        self.motor_output_note.setWordWrap(True)
        safety_layout.addWidget(self.motor_output_checkbox)
        safety_layout.addWidget(self.motor_output_note, 1)
        self.control_layout.addWidget(self.motor_safety_box)

    def process_substitutions(self):
        substitutions = {
            'motor_output_enabled': self.motor_output_checkbox.isChecked(),
            'serial_port': self.serial_port_selector.selected_port(),
        }
        substitutions.update(self.camera_selector.substitutions())
        substitutions.update(self.tag_camera_selector.substitutions())
        if self.estimation_roi is not None:
            substitutions.update({
                'roi_x': self.estimation_roi['x'], 'roi_y': self.estimation_roi['y'],
                'roi_width': self.estimation_roi['w'], 'roi_height': self.estimation_roi['h'],
            })
        return substitutions

    def update_roi_label(self):
        if self.estimation_roi is None:
            self.roi_label.setText('Estimation ROI: choose an area before Start.')
        else:
            roi = self.estimation_roi
            self.roi_label.setText(
                f"Next estimation ROI: x={roi['x']}, y={roi['y']}, "
                f"w={roi['w']}, h={roi['h']}")
        self.roi_label.setToolTip(
            'Top-left pixel x/y and width/height. Applies on next Segment estimation Start. '
            'AprilTag continues to use the full camera image from its separate tag camera.')

    def roi_recording_block_reason(self):
        if getattr(self, 'record_future', None) is not None or getattr(self, 'is_recording', False):
            return 'Finish recording and CSV export before changing estimation ROI.'
        snapshot = getattr(self.node, 'record_status', None)
        if snapshot:
            status, _ = snapshot
            if status.get('state') in ('starting', 'recording', 'stopping', 'exporting'):
                return 'Finish recording and CSV export before changing estimation ROI.'
        return ''

    def roi_edit_block_reason(self):
        reason = self.roi_recording_block_reason()
        if reason:
            return reason
        if self.motor_output_checkbox.isChecked():
            return 'Stop robot motion and disable physical motor output before changing ROI.'
        managed = self.system_manager.processes.get('estimation')
        if managed is None:
            return 'Segment estimation is not configured in System components.'
        discovered = self.node.discovered_node_names()
        if '/segment_estimation_node' in discovered:
            return 'Stop the externally started segment_estimation_node before editing its ROI.'
        if managed.status(discovered) not in ('stopped', 'failed'):
            return 'Stop Segment estimation and wait for STOPPED before editing its ROI.'
        return ''

    def edit_estimation_roi(self):
        reason = self.roi_edit_block_reason()
        if reason:
            QMessageBox.information(self, 'Estimation ROI', reason)
            return
        if self.roi_config_path is None:
            QMessageBox.warning(self, 'Estimation ROI', 'estimation_pkg is not installed.')
            return
        self.roi_editor_active = True
        try:
            dialog = RoiEditorDialog(
                self.node.context,
                self.estimation_roi or {'x': 0, 'y': 0, 'w': 1, 'h': 1},
                self.roi_config_path, parent=self, apply_guard=self.roi_edit_block_reason)
            self.roi_dialog = dialog
            if dialog.exec_() == QDialog.Accepted:
                # Recheck after the nested Qt loop; external nodes may appear.
                reason = self.roi_edit_block_reason()
                if reason:
                    QMessageBox.warning(self, 'Estimation ROI', reason)
                    return
                self.estimation_roi = dict(dialog.selected_roi)
                self.update_roi_label()
                self.node.get_logger().info(
                    f'ROI selected for next Segment estimation Start: {self.estimation_roi}')
        except (OSError, ValueError, RuntimeError) as error:
            QMessageBox.warning(self, 'Estimation ROI', str(error))
        finally:
            if self.roi_dialog is not None:
                self.roi_dialog.deleteLater()
                self.roi_dialog = None
            self.roi_editor_active = False

    def schedule_auto_start(self):
        self.cancel_scheduled_starts()
        discovered = self.node.discovered_node_names()
        for spec in self.system_manager.specs:
            if not self.auto_start_enabled.get(spec.component_id, False):
                continue
            if self.system_manager.processes[spec.component_id].status(
                    discovered) not in ('stopped', 'failed'):
                continue
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.timeout.connect(
                lambda component_id=spec.component_id:
                self.start_component_if_selected(component_id)
            )
            timer.start(int(spec.delay_sec * 1000))
            self.auto_start_timers.append(timer)

    def cancel_scheduled_starts(self):
        for timer in self.auto_start_timers:
            timer.stop()
            timer.deleteLater()
        self.auto_start_timers = []

    def start_component_if_selected(self, component_id):
        if self.auto_start_enabled.get(component_id, False):
            self.start_component(component_id)

    def start_component(self, component_id):
        try:
            if component_id == 'estimation':
                if self.roi_editor_active:
                    raise ValueError('ROI editor is open; start Segment estimation after applying ROI.')
                if self.estimation_roi is None:
                    raise ValueError('Choose an estimation ROI before starting Segment estimation.')
                reason = self.roi_recording_block_reason()
                if reason:
                    raise ValueError(reason)
                if '/segment_estimation_node' in self.node.discovered_node_names():
                    raise ValueError('Stop the externally started segment_estimation_node first.')
            substitutions = self.process_substitutions()
            if component_id == 'serial':
                substitutions['serial_port'] = self.serial_port_selector.port_for_start()
            elif component_id in self.camera_selectors:
                substitutions.update(self.camera_selectors[component_id].for_start())
            started = self.system_manager.start(
                component_id,
                self.node.discovered_node_names(),
                substitutions,
            )
            if started:
                if component_id == 'serial':
                    self.serial_port_selector.mark_started(substitutions['serial_port'])
                elif component_id in self.camera_selectors:
                    self.camera_selectors[component_id].mark_started(substitutions)
                self.node.get_logger().info(
                    f'Starting component: {component_id}'
                    + (f" on {substitutions['serial_port']}" if component_id == 'serial' else '')
                )
        except Exception as error:
            self.node.get_logger().error(
                f'Failed to start {component_id}: {error}'
            )

    def toggle_component(self, component_id):
        managed = self.system_manager.processes[component_id]
        status = managed.status(self.node.discovered_node_names())
        if status in ('running', 'starting', 'unhealthy'):
            self.system_manager.stop(component_id)
        elif status == 'external':
            QMessageBox.information(
                self,
                'Externally managed process',
                f'{managed.spec.label} was not started by this GUI, so the '
                'GUI will not stop it.',
            )
        elif status in ('stopped', 'failed'):
            self.start_component(component_id)

    def stop_all_components(self):
        self.cancel_scheduled_starts()
        self.system_manager.stop_all()

    def update_system_status(self):
        if self.is_closing:
            return
        self.system_manager.poll()
        discovered = self.node.discovered_node_names()
        if '/robot_control' not in discovered:
            self.motor_gate_initialized_for_node = False
        elif not self.motor_gate_initialized_for_node:
            future = self.node.set_motor_output_enabled(
                self.motor_output_checkbox.isChecked()
            )
            if future is not None:
                self.motor_gate_initialized_for_node = True
                future.add_done_callback(
                    lambda completed,
                    value=self.motor_output_checkbox.isChecked():
                    QTimer.singleShot(
                        0,
                        lambda: self.motor_output_parameter_finished(
                            completed,
                            value,
                        ),
                    )
                )
        styles = {
            'running': ('● RUNNING', '#188038'),
            'starting': ('● STARTING', '#e37400'),
            'stopping': ('● STOPPING', '#e37400'),
            'graph_pending': ('● STOPPED*', '#777777'),
            'unhealthy': ('● NO NODE', '#d93025'),
            'external': ('● EXTERNAL', '#1a73e8'),
            'failed': ('● FAILED', '#d93025'),
            'stopped': ('● STOPPED', '#777777'),
        }
        for component_id, widgets in self.component_widgets.items():
            status = self.system_manager.processes[component_id].status(
                discovered
            )
            if component_id == 'serial':
                self.serial_port_selector.set_process_status(status)
            elif component_id in self.camera_selectors:
                self.camera_selectors[component_id].set_process_status(status)
            text, color = styles[status]
            widgets['status'].setText(text)
            widgets['status'].setStyleSheet(
                f'font-weight: 600; color: {color};'
            )
            pending_note = (
                'The GUI-owned process group has exited. Waiting for its ROS '
                'node names to disappear before allowing a restart. A graph '
                'entry alone does not prove another process is running.'
                if status == 'graph_pending' else ''
            )
            widgets['status'].setToolTip(pending_note)
            widgets['button'].setToolTip(pending_note)
            if status in ('running', 'starting', 'unhealthy'):
                widgets['button'].setText('Stop')
                widgets['button'].setEnabled(True)
            elif status == 'stopping':
                widgets['button'].setText('Stopping…')
                widgets['button'].setEnabled(False)
            elif status == 'graph_pending':
                widgets['button'].setText('ROS cleanup…')
                widgets['button'].setEnabled(False)
            elif status == 'external':
                widgets['button'].setText('External')
                widgets['button'].setEnabled(True)
            else:
                widgets['button'].setText('Start')
                widgets['button'].setEnabled(True)
        self.camera_exposure.update_nodes(discovered)
        self.update_data_status_labels()

    def update_data_status_labels(self):
        now = time.monotonic()

        def update_label(label, title, last_received):
            fresh = (
                last_received is not None
                and now - last_received < 1.0
            )
            if fresh:
                label.setText(f'● {title}: receiving')
                label.setStyleSheet('font-weight: 600; color: #188038;')
            else:
                label.setText(f'● {title}: no recent data')
                label.setStyleSheet('font-weight: 600; color: #d93025;')

        if hasattr(self, 'motor_data_status_label'):
            update_label(
                self.motor_data_status_label,
                'motor_state',
                self.node.last_motor_state_time,
            )
        if hasattr(self, 'sensor_data_status_label'):
            newest_sensor_time = None
            sensor_times = [
                self.node.last_fts_data_time,
                self.node.last_loadcell_data_time,
            ]
            valid_times = [value for value in sensor_times if value is not None]
            if valid_times:
                newest_sensor_time = max(valid_times)
            update_label(
                self.sensor_data_status_label,
                'serial sensor data',
                newest_sensor_time,
            )

    def motor_output_toggled(self, enabled):
        if enabled:
            answer = QMessageBox.warning(
                self,
                'Enable physical motor output?',
                'Commands from the GUI will be allowed onto motor_command. '
                'Confirm motor indices, winding signs and travel limits first.',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                self.motor_output_checkbox.blockSignals(True)
                self.motor_output_checkbox.setChecked(False)
                self.motor_output_checkbox.blockSignals(False)
                return

        future = self.node.set_motor_output_enabled(enabled)
        if future is None:
            self.motor_output_note.setText(
                ('ON' if enabled else 'OFF')
                + ' — applies when Robot control is next started'
            )
        else:
            self.motor_gate_initialized_for_node = True
            self.motor_output_note.setText(
                ('ON' if enabled else 'OFF')
                + ' — updating Robot control parameter…'
            )
            future.add_done_callback(
                lambda completed, value=enabled:
                QTimer.singleShot(
                    0,
                    lambda: self.motor_output_parameter_finished(
                        completed, value
                    ),
                )
            )

    def motor_output_parameter_finished(self, future, enabled):
        try:
            response = future.result()
            results = response.results if response is not None else []
            successful = bool(results) and all(
                result.successful for result in results
            )
        except Exception as error:
            successful = False
            self.node.get_logger().error(
                f'Could not update motor output gate: {error}'
            )

        if successful:
            state = 'ENABLED' if enabled else 'DISABLED'
            self.motor_output_note.setText(
                f'{state} — Robot control acknowledged the setting'
            )
            color = '#d93025' if enabled else '#188038'
            self.motor_output_note.setStyleSheet(
                f'font-weight: 600; color: {color};'
            )
        else:
            self.motor_output_note.setText(
                'Parameter update failed; restart Robot control to apply it'
            )

    def init_recorder_ui(self):
        self.record_layout = QVBoxLayout()
        self.record_layout.setSpacing(3)
        self.force_alignment_panel = ForceAlignmentPanel()
        self.force_alignment_panel.layout().setContentsMargins(8, 6, 8, 6)
        self.force_alignment_panel.layout().setSpacing(2)
        self.record_layout.addWidget(self.force_alignment_panel)
        self.record_label = QLabel('Recording')
        self.record_button = QPushButton('Record')
        self.record_button.setStyleSheet('QPushButton {color: green;}')  # 초록색으로 변경
        self.record_button.clicked.connect(self.toggle_record)
        self.record_button.setFixedWidth(110)
        self.record_button.setFixedHeight(28)
        record_actions = QHBoxLayout()
        record_actions.addWidget(self.record_button)
        self.record_label.setWordWrap(True)
        record_actions.addWidget(self.record_label, 1)
        self.record_layout.addLayout(record_actions)
        self.layout_global.addLayout(self.record_layout)

        self.is_recording = False
        self.record_future = None
        self.record_request_time = 0.0
        self.record_error = None
        self.record_timer = QTimer(self)
        self.record_timer.timeout.connect(self.update_record_status)
        self.record_timer.start(200)

    def toggle_record(self):
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        if getattr(self, 'roi_editor_active', False):
            self.record_label.setText('Close the ROI editor before starting recording.')
            return
        try:
            alignment = self.force_alignment_panel.settings()
        except ValueError as error:
            self.record_label.setText(f'Invalid force-axis mapping: {error}')
            return
        self.force_alignment_panel.contact_segment_id.interpretText()
        future = self.node.send_request_record_start(
            alignment, self.force_alignment_panel.contact_segment_id.value())
        if future is None:
            self.record_label.setText('Recorder/settings service unavailable; start Record component first.')
            return
        self.record_future = future
        self.record_error = None
        self.record_request_time = time.monotonic()
        self.record_button.setEnabled(False)
        self.force_alignment_panel.setEnabled(False)
        self.record_label.setText('Applying experiment settings, then requesting recording…')

    def stop_recording(self):
        future = self.node.send_request_record_stop()
        if future is None:
            self.record_label.setText('Recorder service unavailable; recording state unknown.')
            return
        self.record_future = future
        self.record_error = None
        self.record_request_time = time.monotonic()
        self.record_button.setEnabled(False)
        self.record_label.setText('Requesting stop; waiting for bag flush…')

    def update_record_status(self):
        if self.record_future is not None:
            if not self.record_future.done():
                if time.monotonic() - self.record_request_time > 5.0:
                    self.record_label.setText('Waiting for recorder acknowledgement; check Record component.')
                return
            try:
                response = self.record_future.result()
                self.record_label.setText(response.message)
                if not response.success:
                    self.record_error = response.message
                else:
                    # Ignore idle heartbeats from the configuration phase.
                    # Only a lifecycle update after this ACK may unlock UI.
                    self.record_request_time = time.monotonic()
            except Exception as exc:
                self.record_error = f'Record request failed: {exc}'
                self.record_label.setText(self.record_error)
            self.record_future = None
            # An ACK is not a fresh lifecycle status: keep settings locked
            # until that status arrives, except when the request failed.
            self.record_button.setEnabled(bool(self.record_error))
            self.force_alignment_panel.setEnabled(bool(self.record_error) and not self.is_recording)
        snapshot = getattr(self.node, 'record_status', None)
        if not snapshot:
            return
        status, received = snapshot
        if received < self.record_request_time:
            return
        if time.monotonic() - received > 3.0:
            self.record_label.setText('Recorder status stale; check Record component before continuing.')
            self.record_button.setEnabled(False)
            self.force_alignment_panel.setEnabled(False)
            return
        state = status.get('state', 'unknown')
        self.is_recording = state in ('starting', 'recording', 'stopping')
        self.record_button.setText('Exporting…' if state == 'exporting' else ('Stop' if self.is_recording else 'Record'))
        self.record_button.setEnabled(state not in ('stopping', 'exporting'))
        self.force_alignment_panel.setEnabled(not self.is_recording and state != 'exporting')
        if status.get('force_alignment_scope') == 'session':
            self.force_alignment_panel.show_session(status.get('force_alignment'))
            contact_id = status.get('contact_segment_id')
            if type(contact_id) is int and 0 <= contact_id <= 18:
                self.force_alignment_panel.contact_segment_id.setValue(contact_id)
        color = {'recording': 'red', 'starting': '#b9770e',
                 'stopping': '#b9770e', 'exporting': '#b9770e', 'failed': '#b03a2e'}.get(state, 'green')
        if (status.get('data_quality') in ('incomplete', 'unverified')
                or (status.get('live_health') or {}).get('status') == 'warning'):
            color = '#b9770e'
        self.record_button.setStyleSheet(f'QPushButton {{color: {color};}}')
        self.record_label.setText(f"{state}: {status.get('detail', '')}")
        if self.record_error:
            self.record_label.setText(f'{self.record_error} (recorder: {state})')
        self.record_label.setToolTip(status.get('directory', ''))

    def init_motor_ui(self):
        '''
        @ autor DY
        @ note motor subscriber and publisher list
        '''
        self.motor_data_status_label = QLabel('● motor_state: no recent data')
        self.layout_global.addWidget(self.motor_data_status_label)
        self.layout_mode = QHBoxLayout()
        self.layout_mode.setSpacing(12)
        self.label_mode = QLabel('Operation Mode')
        self.checkbox_mode_list = [QCheckBox('Manual'), QCheckBox('Kinematics'), QCheckBox('Dynamics'), QCheckBox('Position'), QCheckBox('Admittance')]
        self.checkbox_mode_list[0].setChecked(False)
        self.checkbox_mode_list[0].clicked.connect(self.checkbox_mode_clicked)
        self.checkbox_mode_list[0].stateChanged.connect(self.disable_mode)
        self.checkbox_mode_list[1].setChecked(True)
        self.checkbox_mode_list[1].clicked.connect(self.checkbox_mode_clicked)
        self.checkbox_mode_list[2].setChecked(False)
        self.checkbox_mode_list[2].clicked.connect(self.checkbox_mode_clicked)
        self.checkbox_mode_list[3].setChecked(False)
        self.checkbox_mode_list[3].clicked.connect(self.checkbox_mode_clicked)
        self.checkbox_mode_list[4].setChecked(False)
        self.checkbox_mode_list[4].clicked.connect(self.checkbox_mode_clicked)
        # self.checkbox_mode_list[1].stateChanged.connect(self.disable_mode)
        self.layout_mode.addWidget(self.label_mode)
        self.layout_mode.addWidget(self.checkbox_mode_list[0])
        self.layout_mode.addWidget(self.checkbox_mode_list[1])
        self.layout_mode.addWidget(self.checkbox_mode_list[2])
        self.layout_mode.addWidget(self.checkbox_mode_list[3])
        self.layout_mode.addWidget(self.checkbox_mode_list[4])
        self.layout_mode.addStretch()
        self.layout_global.addLayout(self.layout_mode)

        self.motor_layout_list = []
        self.motor_state_label_list = []
        self.motor_state_line_edit_list = []
        self.target_wire_length_label_list = []
        self.target_wire_length_line_edit_list = []

        self.motor_pub_label_list = []
        self.motor_pub_line_edit_list = []
        self.motor_pub_button_list = []

        motor_names = ('East', 'West', 'South', 'North')
        for i in range(self.node.numofmotors):  # make label, line_editor, push button for motor control
            self.motor_layout_list.append(QHBoxLayout())

            motor_name = motor_names[i] if i < len(motor_names) else f'#{i}'
            self.motor_state_label_list.append(
                QLabel(f'#{i} {motor_name} a_pos:')
            )
            self.motor_state_line_edit_list.append(QLineEdit('0'))

            self.target_wire_length_label_list.append(QLabel('IK ΔL [mm]'))
            self.target_wire_length_line_edit_list.append(QLineEdit('0.00000'))

            self.motor_pub_label_list.append(QLabel('move(relative)'))
            self.motor_pub_line_edit_list.append(QLineEdit('0'))
            self.motor_pub_button_list.append(QPushButton('Publish'))

            self.motor_pub_button_list[i].clicked.connect(
                lambda checked=False, index=i: self.publish_motion(num=index)
            )

        # @autor DY
        # lambda F for apply the funtion to all the list arugments)
        list(map(lambda x: x.setAlignment(Qt.AlignVCenter | Qt.AlignRight), self.motor_state_label_list))        
        list(map(lambda x: x.setFixedWidth(85), self.motor_state_line_edit_list))
        list(map(lambda x: x.setFixedHeight(26), self.motor_state_line_edit_list))
        list(map(lambda x: x.setReadOnly(True), self.motor_state_line_edit_list))
        list(map(lambda x: x.setAlignment(Qt.AlignVCenter | Qt.AlignRight), self.target_wire_length_label_list))
        list(map(lambda x: x.setFixedWidth(85), self.target_wire_length_line_edit_list))
        list(map(lambda x: x.setFixedHeight(26), self.target_wire_length_line_edit_list))
        list(map(lambda x: x.setReadOnly(True), self.target_wire_length_line_edit_list))
        list(map(lambda x: x.setAlignment(Qt.AlignVCenter | Qt.AlignRight), self.motor_pub_label_list))
        list(map(lambda x: x.setFixedWidth(85), self.motor_pub_line_edit_list))
        list(map(lambda x: x.setFixedHeight(26), self.motor_pub_line_edit_list))
        list(map(lambda x: x.setFixedWidth(80), self.motor_pub_button_list))
        list(map(lambda x: x.setFixedHeight(26), self.motor_pub_button_list))

        for i in range(self.node.numofmotors):
            try:
                self.motor_layout_list[i].addWidget(self.motor_state_label_list[i])
                self.motor_layout_list[i].addWidget(self.motor_state_line_edit_list[i])
                self.motor_layout_list[i].addWidget(self.target_wire_length_label_list[i])
                self.motor_layout_list[i].addWidget(self.target_wire_length_line_edit_list[i])
                self.motor_layout_list[i].addWidget(self.motor_pub_label_list[i])
                self.motor_layout_list[i].addWidget(self.motor_pub_line_edit_list[i])
                self.motor_layout_list[i].addWidget(self.motor_pub_button_list[i])
                self.motor_pub_button_list[i].setEnabled(False)
                self.layout_global.addLayout(self.motor_layout_list[i])
            finally:
                pass

        self.layout_amode = QHBoxLayout()
        self.layout_amode.setSpacing(12)
        self.label_amode = QLabel('Actuation mode')
        self.checkbox_amode_list = [QCheckBox('Absolute'), QCheckBox('Relative')]
        self.checkbox_amode_list[0].setChecked(True)
        self.checkbox_amode_list[0].clicked.connect(self.checkbox_amode_clicked)
        self.checkbox_amode_list[1].setChecked(False)
        self.checkbox_amode_list[1].clicked.connect(self.checkbox_amode_clicked)
        self.layout_amode.addWidget(self.label_amode)
        self.layout_amode.addWidget(self.checkbox_amode_list[0])
        self.layout_amode.addWidget(self.checkbox_amode_list[1])
        self.layout_amode.addStretch()
        self.layout_global.addLayout(self.layout_amode)

        self.motor_kinematics_layout = QVBoxLayout()
        self.motor_kinematics_layout.setSpacing(3)
        self.motor_kinematics_label_list = []
        self.motor_kinematics_label_list.append(QLabel("Move Tip (degree) | Pan (E-W, q2)"))
        self.motor_kinematics_label_list.append(QLabel("Move Tip (degree) | Tilt (S-N, q1 about Base Z)"))

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
        list(map(lambda x: x.setFixedWidth(85), self.motor_kinematics_line_edit_list))
        list(map(lambda x: x.setFixedHeight(26), self.motor_kinematics_line_edit_list))
        self.motor_kinematics_button = QPushButton('Publish')
        self.motor_kinematics_button.clicked.connect(self.publish_motion)
        self.motor_kinematics_button.setFixedWidth(110)
        self.motor_kinematics_button.setFixedHeight(55)

        self.motor_kinematics_layout_fin = QHBoxLayout()
        self.motor_kinematics_layout_fin.addLayout(self.motor_kinematics_layout)
        self.motor_kinematics_layout_fin.addWidget(self.motor_kinematics_button)
        self.layout_global.addLayout(self.motor_kinematics_layout_fin)

        self.sine_motion_panel = SineMotionPanel(
            self.node, kinematics_selected=lambda: self.checkbox_mode_list[1].isChecked())
        self.layout_global.addWidget(self.sine_motion_panel)

        # self.motor_kinematics_label = QLabel("Move Tip(Degree) | Tilt")
        # self.motor_kinematics_line_edit = QLineEdit('0')
        # self.motor_kinematics_button = QPushButton('Publish')
        # self.motor_kinematics_button.clicked.connect(self.publish_motion)
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

        self.LPF_parameter_label = QLabel('Weight:')
        self.LPF_parameter_label.setToolTip('Low-pass filter weight (0 to 1).')
        self.LPF_parameter_label.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.LPF_parameter = QLineEdit('0.5')
        self.LPF_parameter.setFixedWidth(60)
        self.LPF_parameter.setFixedHeight(26)

        self.MAF_parameter_label = QLabel('Buffer size:')
        self.MAF_parameter_label.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.MAF_parameter = QLineEdit('5')
        self.MAF_parameter.setFixedWidth(60)
        self.MAF_parameter.setFixedHeight(26)

        self.layout_LPF.addWidget(self.LPF_parameter_label)
        self.layout_LPF.addWidget(self.LPF_parameter)
        self.layout_MAF.addWidget(self.MAF_parameter_label)
        self.layout_MAF.addWidget(self.MAF_parameter)

        self.checkbox_filter_list = [QCheckBox('LPF'), QCheckBox('Moving average')]
        self.checkbox_filter_list[0].setToolTip('Enable the weighted low-pass filter.')
        self.checkbox_filter_list[1].setToolTip('Enable the moving-average filter.')
        self.checkbox_filter_list[0].setChecked(False)
        self.checkbox_filter_list[1].setChecked(False)
        self.layout_LPF.addWidget(self.checkbox_filter_list[0])
        self.layout_MAF.addWidget(self.checkbox_filter_list[1])
        self.layout_LPF.setAlignment(self.checkbox_filter_list[0], Qt.AlignRight)
        self.layout_MAF.setAlignment(self.checkbox_filter_list[1], Qt.AlignRight)
        filter_row = QHBoxLayout()
        filter_row.addLayout(self.layout_LPF)
        filter_row.addSpacing(16)
        filter_row.addLayout(self.layout_MAF)
        self.layout_global.addLayout(filter_row)

        # self.checkbox_amode_list[0].clicked.connect(self.checkbox_amode_clicked)

        # self.filter_checkbox = []

    def init_set_zero_button(self):
        self.zero_button = QPushButton('Set sensor zero')
        self.zero_button.setToolTip(
            'Average current force/torque sensor readings via '
            '/serial_data/set_zero. This does not home or zero any motor.'
        )
        self.zero_button.clicked.connect(self.request_set_zero)
        self.zero_button.setMinimumHeight(36)
        self.zero_button_layout = QVBoxLayout()
        self.zero_button_layout.addWidget(self.zero_button)
        self.layout_global.addLayout(self.zero_button_layout)

    def init_fts_ui(self):
        '''
        @ autor DY
        @ note force-torque sensor list for monitoring
        '''

        self.sensor_data_status_label = QLabel(
            '● serial sensor data: no recent data'
        )
        self.sensor_data_status_label.setWordWrap(True)
        self.layout_global.addWidget(self.sensor_data_status_label)
        self.fts_sub_label_list = []
        self.fts_sub_line_edit_list = []
        self.fts_sub_layout_list = []
        for axis in ('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz'):
            label = QLabel(f'FTS {axis}')
            label.setToolTip(f'Force/torque sensor: {axis}')
            self.fts_sub_label_list.append(label)

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
        """Show every configured loadcell channel in message-array order."""
        self.lc_sub_label_list = []
        self.lc_sub_line_edit_list = []
        self.lc_sub_layout_list = []
        for i in range(self.node.num_loadcells):
            label = QLabel(f'Loadcell #{i + 1} (g)')
            value = QLineEdit('—')
            value.setReadOnly(True)
            row = QHBoxLayout()
            row.addWidget(label)
            row.addWidget(value)
            self.lc_sub_label_list.append(label)
            self.lc_sub_line_edit_list.append(value)
            self.lc_sub_layout_list.append(row)
            self.layout_global.addLayout(row)

    def init_timer(self):
        '''
        @ author DY
        @ note Timer objects
        '''
        try:
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
            self.timer_filter_state = QTimer(self)
            self.timer_filter_state.timeout.connect(self.update_filter_state)
            self.timer_filter_state.start(500)

            # ros node
            self.timer_ros_node = QTimer(self)
            self.timer_ros_node.timeout.connect(self.node_spin_once)
            self.timer_ros_node.start(5)

            self.timer_system_status = QTimer(self)
            self.timer_system_status.timeout.connect(self.update_system_status)
            self.timer_system_status.start(500)
            self.update_system_status()
        except Exception as e:
            self.node.get_logger().warning(f'F:init_timer() -> {e}')

    def init_fts_plot(self):
        try:
            # Matplotlib graph
            self.figure, self.fts_axes = plt.subplots(2, 1, sharex=True)
            self.figure.subplots_adjust(left=0.18, right=0.97, bottom=0.12,
                                        top=0.97, hspace=0.15)
            self.canvas = FigureCanvas(self.figure)
            self.layout_global.addWidget(self.canvas)
            self.data_y = np.zeros((6, 50))  # 초기 데이터 설정 (6개의 데이터, 각각 100개의 요소)
            self.data_x = [i for i in range(len(self.data_y))]

            # 그래프 초기화
            labels = ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')
            self.lines = [self.fts_axes[i // 3].plot([], [], label=label)[0]
                          for i, label in enumerate(labels)]
            # Avoid Matplotlib's default 'best' location following live traces.
            for axis, label in zip(self.fts_axes, ('Force', 'Torque')):
                axis.set_ylabel(label)
                axis.legend(loc='upper right', ncol=3, fontsize=8)
                axis.tick_params(labelsize=8)
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
            # self.node.send_request_set_control_mode(mode=0)
            self.checkbox_mode_list[1].setChecked(False)
            self.checkbox_mode_list[2].setChecked(False)
            self.checkbox_mode_list[3].setChecked(False)
            self.checkbox_mode_list[4].setChecked(False)

        # kinematics
        elif sender == self.checkbox_mode_list[1] and self.checkbox_mode_list[1].isChecked():
            self.node.get_logger().info("Setting parameter...")
            self.node.send_request_set_control_mode(mode=ControlMode.kKinematics)
            self.checkbox_mode_list[0].setChecked(False)
            self.checkbox_mode_list[2].setChecked(False)
            self.checkbox_mode_list[3].setChecked(False)
            self.checkbox_mode_list[4].setChecked(False)
            self.motor_kinematics_label_list[0].setText("Move Tip (degree) | Pan (E-W, q2)")
            self.motor_kinematics_label_list[1].setText("Move Tip (degree) | Tilt (S-N, q1 about Base Z)")

        # dynamics
        elif sender == self.checkbox_mode_list[2] and self.checkbox_mode_list[2].isChecked():
            self.node.get_logger().info("Setting parameter...")
            self.node.send_request_set_control_mode(mode=ControlMode.kDynamics)
            self.checkbox_mode_list[0].setChecked(False)
            self.checkbox_mode_list[1].setChecked(False)
            self.checkbox_mode_list[3].setChecked(False)
            self.checkbox_mode_list[4].setChecked(False)
            self.motor_kinematics_label_list[0].setText("Move Tip (degree) | Pan (E-W, q2)")
            self.motor_kinematics_label_list[1].setText("Move Tip (degree) | Tilt (S-N, q1 about Base Z)")

        # position
        elif sender == self.checkbox_mode_list[3] and self.checkbox_mode_list[3].isChecked():
            self.node.get_logger().info("Setting parameter...")
            self.node.send_request_set_control_mode(mode=ControlMode.kPosition)
            self.checkbox_mode_list[0].setChecked(False)
            self.checkbox_mode_list[1].setChecked(False)
            self.checkbox_mode_list[2].setChecked(False)
            self.checkbox_mode_list[4].setChecked(False)
            self.motor_kinematics_label_list[0].setText("Move Tip (m) | y-axis")
            self.motor_kinematics_label_list[1].setText("Move Tip (m) | z-axis")

        # admittance
        elif sender == self.checkbox_mode_list[4] and self.checkbox_mode_list[4].isChecked():
            self.node.get_logger().info("Setting parameter...")
            self.node.send_request_set_control_mode(mode=ControlMode.kAdmittance)
            self.checkbox_mode_list[0].setChecked(False)
            self.checkbox_mode_list[1].setChecked(False)
            self.checkbox_mode_list[2].setChecked(False)
            self.checkbox_mode_list[3].setChecked(False)
            self.motor_kinematics_label_list[0].setText("Move Tip (m) | y-axis")
            self.motor_kinematics_label_list[1].setText("Move Tip (m) | z-axis")

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

    def publish_motion(self, num=0):
        try:
            if (self.checkbox_mode_list[0].isChecked() and
                    len(self.node.motor_state.actual_position) == 0):
                self.node.get_logger().warning(f'motor_state does not exit. Check the connection')
                return

            else:

                # Mode : Manual operation of each motors
                if self.checkbox_mode_list[0].isChecked():
                    if self.node.opmode == 0x08:   # CSP
                        # cmd_val = self.node.motor_state.actual_position[num] + (int(self.motor_pub_line_edit_list[num].text()))
                        cmd_val = int(self.motor_pub_line_edit_list[num].text())
                        print('python', cmd_val)
                        response = self.node.send_request_move_motor_direct(idx=num, tp=cmd_val, tvp=50)
                    elif self.node.opmode == 0x09: # CSV
                        cmd_val = int(self.motor_pub_line_edit_list[num].text())
                    self.node.get_logger().info(f'motor #{num} -> {cmd_val} command update {response}.')

                # Mode : Kinematics
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
                    if self.checkbox_amode_list[0].isChecked(): # Absolute
                        response = self.node.send_request_move_tool_angle_dynamics(pan=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                                    tilt=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                                    mode=0)
                    elif self.checkbox_amode_list[1].isChecked():
                        response = self.node.send_request_move_tool_angle_dynamics(pan=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                                    tilt=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                                    mode=1)

                # Mode : Position
                elif self.checkbox_mode_list[3].isChecked():
                    if self.checkbox_amode_list[0].isChecked(): # Absolute
                        response = self.node.send_request_set_goal_position(x=0.0,
                                                                            y=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                            z=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                            ref='absolute')
                    elif self.checkbox_amode_list[1].isChecked():
                        response = self.node.send_request_set_goal_position(x=0.0,
                                                                            y=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                            z=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                            ref='relative')
                # Mode : Admittance
                elif self.checkbox_mode_list[4].isChecked():
                    if self.checkbox_amode_list[0].isChecked(): # Absolute
                        response = self.node.send_request_set_goal_position(x=0.0,
                                                                            y=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                            z=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                            ref='absolute')
                    elif self.checkbox_amode_list[1].isChecked():
                        response = self.node.send_request_set_goal_position(x=0.0,
                                                                            y=float(self.motor_kinematics_line_edit_list[0].text()),
                                                                            z=float(self.motor_kinematics_line_edit_list[1].text()),
                                                                            ref='relative')

        except Exception as e:
            self.node.get_logger().warning(f'F:publish_motion() -> {e}')
            return

    def request_set_zero(self):
        try:
            response = self.node.send_request_set_zero()
        except Exception as e:
            self.node.get_logger().warning(f'F:set_zero() -> {e}')

    def node_spin_once(self):
        try:
            if rclpy.ok():
                # Never block Qt's event loop while waiting for ROS data.
                # With no motor/serial connection, the default timeout (None)
                # would freeze repainting and leave the window black.
                rclpy.spin_once(self.node, timeout_sec=0.0)
        except rclpy.exceptions.RCLError:
            print("⚠️ ROS shutdown 이후 spin_once 호출 방지됨")
            # rclpy.shutdown()

    def update_motor_state(self):
        for i in range(self.node.numofmotors):
            try:
                if i < len(self.node.motor_state.actual_position):
                    self.motor_state_line_edit_list[i].setText(
                        str(self.node.motor_state.actual_position[i])
                    )
                if i < len(self.node.target_wire_length):
                    self.target_wire_length_line_edit_list[i].setText(
                        f'{self.node.target_wire_length[i]:+.5f}'
                    )
            except Exception as e:
                self.node.get_logger().warning(f'F:update_motor_state({i}) -> {e}')
        return

    def update_fts(self):
        try:
            self.fts_sub_line_edit_list[0].setText(str(self.node.fts_data.wrench.force.x))
            self.fts_sub_line_edit_list[1].setText(str(self.node.fts_data.wrench.force.y))
            self.fts_sub_line_edit_list[2].setText(str(self.node.fts_data.wrench.force.z))
            self.fts_sub_line_edit_list[3].setText(f'{self.node.fts_data.wrench.torque.x:.1f}')
            self.fts_sub_line_edit_list[4].setText(f'{self.node.fts_data.wrench.torque.y:.1f}')
            self.fts_sub_line_edit_list[5].setText(f'{self.node.fts_data.wrench.torque.z:.1f}')
        except Exception as e:
            # self.node.get_logger().warning(f'F:update_fts() -> {e}')
            pass
        return 1

    def update_loadcell(self):
        values = self.node.loadcell_data.stress
        for i, field in enumerate(self.lc_sub_line_edit_list):
            # Clear missing channels instead of silently retaining old values.
            field.setText(str(values[i]) if i < len(values) else '—')
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
                for group, axis in enumerate(self.fts_axes):
                    values = self.data_y[group * 3:(group + 1) * 3]
                    finite = values[np.isfinite(values)]
                    if finite.size:
                        min_val, max_val = np.min(finite), np.max(finite)
                        margin = max((max_val - min_val) * 0.1, 0.1)
                        axis.set_ylim(min_val - margin, max_val + margin)
                    axis.set_xlim(0, self.data_y.shape[1] - 1)

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

        try:
            if rclpy.ok():
                self.node.data_filter_setting_publisher.publish(msg)
        except Exception as e:
            # except rclpy.exceptions.RCLError as e:
            print(f"❗ update_filter_state publish 에러: {e}")

    def closeEvent(self, event):
        if self.is_closing:
            event.accept()
            return
        self.is_closing = True
        try:
            self.cancel_scheduled_starts()
            if self.roi_dialog is not None:
                self.roi_dialog.reject()
            # Qt may dispatch a pending status timer after closeEvent returns.
            # Stop every GUI timer before destroying the ROS node handle.
            for timer in self.findChildren(QTimer):
                timer.stop()
            if hasattr(self, 'animation') and self.animation.event_source:
                self.animation.event_source.stop()
            self.image_preview.stop()
            self.system_manager.stop_all()
            if rclpy.ok():
                rclpy.shutdown()
            self.node.destroy_node()
        except Exception as e:
            print(f"❗ closeEvent 에러: {e}")
        finally:
            event.accept()


def main():

    rclpy.init(args=None)
    node = GUINode()
    app = QApplication(sys.argv)
    gui = MyGUI(node)
    signal.signal(signal.SIGINT, lambda *_: gui.close())
    signal.signal(signal.SIGTERM, lambda *_: gui.close())
    gui.showMaximized()

    exit_code = app.exec_()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
