import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
# from moveit_configs_utils import MoveItConfigsBuilder


from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # moveit_config = (
    #     MoveItConfigsBuilder("annin_ar4_description")
    #     .robot_description(file_path="urdf/ar.urdf.xacro")
    #     .to_moveit_configs()
    # )
        
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
        [FindPackageShare("annin_ar4_moveit_config"), "rviz", "servo_rviz_config.rviz"])
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

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_description"), "urdf", "ar.urdf.xacro"
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "include_gripper:=",
        include_gripper,
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"), "srdf",
            "ar.srdf.xacro"
        ]),
        " ",
        "name:=",
        ar_model_config,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "include_gripper:=",
        include_gripper,
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Get parameters for the Servo node
    servo_yaml = load_yaml("annin_ar4_moveit_config", "config/servo_config.yaml")
    servo_params = {"annin_ar4_moveit_config": servo_yaml}

    # rviz with moveit configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            # planning_pipeline_config,
            # robot_description_kinematics,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("annin_ar4_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ar_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ar_arm_controller", "-c", "/controller_manager"],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
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

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
    )

    nodes_to_start = [rviz_node,
                      ros2_control_node,
                      joint_state_broadcaster_spawner,
                      ar_arm_controller_spawner,
                      servo_node,
                      container,
                      ]
    return LaunchDescription(declared_arguments + nodes_to_start)
