import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    include_gripper = LaunchConfiguration("include_gripper")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Make MoveIt use simulation time. This is needed " +
            "for trajectory planing in simulation.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for AR4 tf_tree",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "include_gripper",
            default_value="False",
            description="Run the servo gripper",
            choices=["True", "False"],
        ))
    rviz_config_file_default = PathJoinSubstitution(
        [FindPackageShare("annin_ar4_moveit_config"), "rviz", "moveit.rviz"])
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=rviz_config_file_default,
            description="Full path to the RViz configuration file to use",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "ar_model",
            default_value="mk2",
            choices=["mk1", "mk2", "mk3"],
            description="Model of AR4",
        ))

    urdf_folder = os.path.join(get_package_share_directory("gorm_arm"), "urdf")
    urdf = os.path.join(urdf_folder, "gorm_arm.urdf")
    robot_description_values = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    robot_description = {'robot_description': robot_description_values}

    # MoveIt Configuration
    srdf_folder = os.path.join(get_package_share_directory("gorm_arm"), "srdf")
    srdf = os.path.join(srdf_folder, "gorm_arm.srdf")
    with open(srdf, 'r') as f:
        semantic_content_values = f.read()
    semantic_content = {'robot_description_semantic': semantic_content_values}


    robot_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "gorm_arm",
            os.path.join("config", "kinematics.yaml"),
        )
    }

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

    # Trajectory Execution Configuration
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

    # Start the actual move_group node/action server
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
                "use_sim_time": use_sim_time
            },
        ],
    )

    # rviz with moveit configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            semantic_content,
            planning_pipeline_config,
            robot_kinematics,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]
    return LaunchDescription(declared_arguments + nodes_to_start)
