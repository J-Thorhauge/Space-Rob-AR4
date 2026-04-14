import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    run_gripper = LaunchConfiguration("run_gripper")
    gripper = LaunchConfiguration("gripper")
    run_ph = LaunchConfiguration("run_ph")
    run_camera = LaunchConfiguration("run_camera")

    # Get the robot description from the xacro file
    urdf_folder = os.path.join(get_package_share_directory("gorm_arm"), "urdf")
    urdf = os.path.join(urdf_folder, "gorm_arm.urdf")
    robot_description_values = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    robot_description = {'robot_description': robot_description_values}

    # Get the controller configuration file
    joint_controllers_cfg = PathJoinSubstitution([
        FindPackageShare("gorm_arm"), 
        "config", 
        "controllers.yaml",
    ])

    # This file is useless, it gets immediately overridden by the other config
    update_rate_config_file = PathJoinSubstitution([
        FindPackageShare("gorm_arm"),
        "config",
        "controller_update_rate.yaml",
    ])

    # Also useless, just a partial controller config
    controllers_folder = os.path.join(get_package_share_directory("gorm_arm"), "config")
    ros2_controllers_path = os.path.join(controllers_folder, "ros2_controllers.yaml")

    # Launch the controller manager node, which will load the robot description and the controllers
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            joint_controllers_cfg,
            # ros2_controllers_path
        ],
        remappings=[('~/robot_description', 'robot_description')], #This remap should not be necessary
        output="screen",
    )

    # Spawn the joint trajectory controller (Is this necessary or is it spawned by the previous launch?)
    spawn_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "1000",
        ],
    )

    # Gripper controller node (connects to the Teensy via usb to control the servo gripper)
    gripper_controller = Node(
        package="gorm_arm",
        executable="gripper_interface_node",
        parameters=[{"gripper": gripper}],
        condition=IfCondition(run_gripper),
    )

    # pH controller node (connects to the pH device via usb and publishes the readings)
    ph_controller = Node(
        package="gorm_arm",
        executable="ph_interface_node",
        condition=IfCondition(run_ph),
    )

    # Camera controller node (connects to the camera device via usb and publishes the frames)
    camera_controller = Node(
        package="gorm_arm",
        executable="camera_interface_node",
        condition=IfCondition(run_camera),
    )

    # Robot state publisher node
    # Not sure what the difference is between this and the joint state broadcaster but,
    # I think this one publishes the robot state to tf and joint states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state broadcaster node
    # Again not sure what the difference is between this and the robot state publisher but,
    # I think this one publishes the joint states to tf and /joint_states
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "1000",
        ],
    )

    # Add all nodes to the launch description
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
            "gripper",
            default_value="none",
            description="Which gripper to run gripper",
            choices=["big", "small", "none"],
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "run_ph",
            default_value="False",
            description="Run the pH device",
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
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(gripper_controller)
    ld.add_action(ph_controller)
    ld.add_action(camera_controller)
    ld.add_action(LogInfo(msg=['Gripper value: ', gripper]))
    return ld
