"""Private Isaac ROS 3.1 component for the offline comparison harness only."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    prefix = '/apriltag_backend_compare'
    component = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='detector', namespace=prefix + '/isaac',
        parameters=[{'size': 0.017, 'max_tags': 64, 'tile_size': 4}],
        remappings=[('image', prefix + '/image'), ('camera_info', prefix + '/camera_info'),
                    ('tag_detections', prefix + '/detections'),
                    ('tf', prefix + '/tf'), ('/tf', prefix + '/tf'),
                    ('tf_static', prefix + '/tf_static'), ('/tf_static', prefix + '/tf_static')],
    )
    return LaunchDescription([ComposableNodeContainer(
        name='container', namespace=prefix + '/isaac', package='rclcpp_components',
        executable='component_container_mt', composable_node_descriptions=[component],
        output='screen')])
