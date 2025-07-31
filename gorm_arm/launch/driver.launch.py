import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    run_gripper = LaunchConfiguration("run_gripper")
    run_camera = LaunchConfiguration("run_camera")
    urdf_folder = os.path.join(get_package_share_directory("gorm_arm"), "urdf")
    urdf = os.path.join(urdf_folder, "gorm_arm.urdf")
    robot_description_values = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    robot_description = {'robot_description': robot_description_values}
    

    joint_controllers_cfg = PathJoinSubstitution([
        FindPackageShare("gorm_arm"), 
        "config", 
        "controllers.yaml",
    ])

    update_rate_config_file = PathJoinSubstitution([
        FindPackageShare("gorm_arm"),
        "config",
        "controller_update_rate.yaml",
    ])

    controllers_folder = os.path.join(get_package_share_directory("gorm_arm"), "config")
    ros2_controllers_path = os.path.join(controllers_folder, "ros2_controllers.yaml")

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            joint_controllers_cfg,
            # ros2_controllers_path
        ],
        remappings=[('~/robot_description', 'robot_description')],
        output="screen",
    )

    spawn_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "100",
        ],
    )

    gripper_controller = Node(
        package="gorm_arm",
        executable="gripper_interface_node",
        condition=IfCondition(run_gripper),
    )

    camera_controller = Node(
        package="gorm_arm",
        executable="camera_interface_node",
        condition=IfCondition(run_camera),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "100",
        ],
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port to connect to the robot",
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "calibrate",
            default_value="True",
            description="Calibrate the robot on startup",
            choices=["True", "False"],
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "run_gripper",
            default_value="True",
            description="Run the servo gripper",
            choices=["True", "False"],
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "run_camera",
            default_value="True",
            description="Run the gripper camera",
            choices=["True", "False"],
        ))

    ld.add_action(controller_manager_node)
    ld.add_action(spawn_joint_controller)
    ld.add_action(gripper_controller)
    ld.add_action(camera_controller)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster)
    return ld
