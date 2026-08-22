from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    description_share = get_package_share_directory('pendulum_description')
    urdf_path = os.path.join(description_share, 'urdf', 'pendulum.urdf')
    rviz_config_path = os.path.join(description_share, 'rviz', 'pendulum.rviz')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = ParameterValue(urdf_file.read(), value_type=str)

    return LaunchDescription([
        Node(package="pendulum_sim", executable="pen_sim"),
        Node(package="pendulum_controller", executable="lqr"),
        Node(package="pendulum_description", executable="joint_state_bridge"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{'robot_description': robot_description}]),
        Node(package="rviz2", executable="rviz2", arguments=['-d', rviz_config_path])
    ])
