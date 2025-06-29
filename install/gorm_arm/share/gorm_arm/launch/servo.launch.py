import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
)

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Get URDF
    urdf_folder = os.path.join(get_package_share_directory("gorm_arm"), "urdf")
    urdf = os.path.join(urdf_folder, "gorm_arm.urdf")
    robot_description_values = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    robot_description = {'robot_description': robot_description_values}

    # Get SRDF
    srdf_folder = os.path.join(get_package_share_directory("gorm_arm"), "srdf")
    srdf = os.path.join(srdf_folder, "gorm_arm.srdf")
    with open(srdf, 'r') as f:
        semantic_content_values = f.read()
    semantic_content = {'robot_description_semantic': semantic_content_values}

    # Get kinematics
    robot_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "gorm_arm",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    # Get parameters for the Servo node
    servo_yaml = load_yaml("gorm_arm", "config/gorm_arm_real_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("gorm_arm"),
            "config/joint_limits.yaml"
        ]),
        allow_substs=True,
    )

    # Planning Configuration
    ompl_planning_yaml = load_yaml("gorm_arm",
                                   "config/ompl_planning.yaml")
    pilz_planning_yaml = load_yaml("gorm_arm",
                                   "config/pilz_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "pilz",
        "planning_pipelines": ["ompl", "pilz"],
        "ompl": ompl_planning_yaml,
        "pilz": pilz_planning_yaml,
    }

    moveit_controller_manager = {
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("gorm_arm"),
            "config/controllers_moveit.yaml"
        ]),
        allow_substs=True,
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # Added due to https://github.com/moveit/moveit2_tutorials/issues/528
        "publish_robot_description_semantic": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            semantic_content,
            robot_kinematics,
            joint_limits,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controller_manager,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {
                "use_sim_time": False
            },
        ],
    )

    # RViz
    rviz_folder = os.path.join(
            get_package_share_directory("gorm_arm"), "rviz")
    rviz_config = os.path.join(rviz_folder, "rviz_config.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            semantic_content,
            robot_kinematics,
        ],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # ComposableNode(
            #     package="robot_state_publisher",
            #     plugin="robot_state_publisher::RobotStatePublisher",
            #     name="robot_state_publisher",
            #     parameters=[robot_description],
            # ),
            # ComposableNode(
            #     package="tf2_ros",
            #     plugin="tf2_ros::StaticTransformBroadcasterNode",
            #     name="static_tf2_broadcaster",
            #     parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            # ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            semantic_content,
            robot_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([
        rviz_node,
        # ros2_control_node,
        # joint_state_broadcaster_spawner,
        # gorm_arm_controller_spawner,
        move_group_node,
        servo_node,
        container
    ])