import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
)

def generate_launch_description():

    urdf_folder = os.path.join(get_package_share_directory("gorm_arm"), "urdf")
    urdf = os.path.join(urdf_folder, "fake_gorm_arm.urdf")
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    srdf_folder = os.path.join(get_package_share_directory("gorm_arm"), "srdf")
    srdf = os.path.join(srdf_folder, "gorm_arm.srdf")
    with open(srdf, 'r') as f:
        semantic_content = f.read()

    rviz_folder = os.path.join(
            get_package_share_directory("gorm_arm"), "rviz")
    rviz_config = os.path.join(rviz_folder, "rviz_config.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{
            'robot_description': robot_description,
            'robot_description_semantic': semantic_content,
        }],
    )

    return LaunchDescription([
        rviz_node
        # ros2_control_node,
        # joint_state_broadcaster_spawner,
        # panda_arm_controller_spawner,
        # servo_node,
        # robot_state_publisher
        # tf2_ros
        # moveit_servo
        # joy
    ])