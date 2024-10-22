import serial
import time


import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Header
from geometry_msgs.msg import WrenchStamped
from custom_interfaces.msg import MotorCommand

import os

usb_device_path = "/dev/ttyACM0"  # USB 장치의 경로에 맞게 변경하세요

# USB 장치에 대한 권한 변경 명령어
command = f"sudo chmod 666 {usb_device_path}"

class FTS(Node):

    def __init__(self):
        super().__init__('fts')
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        # self.declare_parameter('')
        # self.add_on_set_parameters_callback(self.update_parameter)

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.fts_publisher = self.create_publisher(
            WrenchStamped,
            'fts_data',
            QOS_RKL10V
        )

        self.serial_port = '/dev/ttyACM0'  # 사용하는 시리얼 포트(COM 포트)를 지정하세요.
        self.baudrate = 115200  # 아두이노와 통신하는 속도

        # self.force3d = list()   # empty list
        self.force3d = [0 for i in range(3)]   # empty list [0, 0, 0]
        self.torque3d = [0 for i in range(3)]   # empty list [0, 0, 0]
        
    # def update_parameter(self, params):
    #     for param in params:
    #         if param.name = 
        
        self.get_logger().info('fts node is up.')
        self.read_serial_data(self.serial_port, self.baudrate)

    def publishall(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fts'
        msg.wrench.force.x = self.force3d[0]
        msg.wrench.force.y = self.force3d[1]
        msg.wrench.force.z = self.force3d[2]
        msg.wrench.torque.x = self.torque3d[0]
        msg.wrench.torque.y = self.torque3d[1]
        msg.wrench.torque.z = self.torque3d[2]

        self.fts_publisher.publish(msg)
        
    def read_serial_data(self, serial_port, baudrate):
        ser = serial.Serial(serial_port, baudrate, timeout=1)
        try:
            while True:
                # 시리얼 데이터 읽기
                try:
                    # serial_data = ser.readline()  # binary(ASCII)
                    serial_data = ser.readline().decode('utf-8', errors='ignore').rstrip()
                except UnicodeDecodeError as e:
                    print(f"Error decoding data: {e}")
                    self.get_logger().warning('Error decoding data')
                    serial_data = ''

                # 수신된 데이터가 비어있지 않으면 출력
                if serial_data:
                    parsing_data = self.parse_serial_data(serial_data)
                    if parsing_data == None:
                        continue
                    else:
                        try:
                            self.force3d = parsing_data[0:3]
                            self.torque3d = parsing_data[3:6]
                            self.publishall()
                        except ValueError:
                            self.get_logger().warning('Error while convert string to float')
                            pass
                        
        except KeyboardInterrupt:
            self.get_logger().warning('Keyboard Interrupt')
        finally:
            # 시리얼 포트 닫기
            ser.close()
    
    def parse_serial_data(self, str: str):
        stx = '/'
        etx = ';'
        sidx = str.find(stx)
        eidx = str.find(etx)

        if sidx == -1 or eidx == -1 or sidx >= eidx:
            return None
        try:
            data_part = str[sidx+1:eidx]
            data_list = data_part.split(',')
            data_list_float = [float(item) for item in data_list]
            return data_list_float
        except Exception as e:
            self.get_logger().warning('Error while parsing input string: %e', e)
            return None
    
    

def main(args=None):
    rclpy.init(args=args)
    try:
        fts = FTS()
        try:
            rclpy.spin(fts)
        except KeyboardInterrupt:
            fts.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            fts.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()