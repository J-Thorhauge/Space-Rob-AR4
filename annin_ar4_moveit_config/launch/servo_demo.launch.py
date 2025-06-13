import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder


from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
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

    # Command-line arguments
    db_arg = DeclareLaunchArgument("db",
                                   default_value="False",
                                   description="Database flag")
    ar_model_arg = DeclareLaunchArgument("ar_model",
                                         default_value="mk2",
                                         choices=["mk1", "mk2", "mk3"],
                                         description="Model of AR4")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix_arg = DeclareLaunchArgument("tf_prefix",
                                          default_value="",
                                          description="Prefix for AR4 tf_tree")
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "urdf",
            "fake_ar.urdf.xacro",
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "tf_prefix:=",
        tf_prefix,
    ])
    robot_description = {"robot_description": robot_description_content}

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
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    # Get parameters for the Servo node
    servo_yaml = load_yaml("annin_ar4_moveit_config", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("annin_ar4_moveit_config"), "rviz")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            # robot_description_kinematics,
            # planning_pipeline_config,
        ],
    )
    # # Publish TF
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    # )

    # ros2_control using FakeSystem as hardware
    ros2_controllers = ParameterFile(PathJoinSubstitution(
        [FindPackageShare("annin_ar4_driver"), "config", "controllers.yaml"]),
                                     allow_substs=True)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers,
            {
                "tf_prefix": tf_prefix
            },
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
        ],
    )

    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )


    # tf2_ros
    static_tf_broadcaster = Node(
        package="tf2_ros",
        executable="tf2_ros::StaticTransformBroadcasterNode",
        name="static_tf2_broadcaster",
        output="both",
        parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
    )

    # moveit_servo
    moveit_servo = Node(
        package="moveit_servo",
        executable="moveit_servo::JoyToServoPub",
        name="controller_to_servo_node",
        output="both",
    )

    # moveit_servo
    joy = Node(
        package="joy",
        executable="joy::joy",
        name="joy_node",
        output="both",
    )

    # # Launch as much as possible in components
    # container = ComposableNodeContainer(
    #     name="moveit_servo_demo_container",
    #     namespace="/",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         # Example of launching Servo as a node component
    #         # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
    #         # ComposableNode(
    #         #     package="moveit_servo",
    #         #     plugin="moveit_servo::ServoServer",
    #         #     name="servo_server",
    #         #     parameters=[
    #         #         servo_params,
    #         #         moveit_config.robot_description,
    #         #         moveit_config.robot_description_semantic,
    #         #     ],
    #         # ),
    #         ComposableNode(
    #             package="robot_state_publisher",
    #             plugin="robot_state_publisher::RobotStatePublisher",
    #             name="robot_state_publisher",
    #             parameters=[robot_description],
    #         ),
    #         ComposableNode(
    #             package="tf2_ros",
    #             plugin="tf2_ros::StaticTransformBroadcasterNode",
    #             name="static_tf2_broadcaster",
    #             parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
    #         ),
    #         ComposableNode(
    #             package="moveit_servo",
    #             plugin="moveit_servo::JoyToServoPub",
    #             name="controller_to_servo_node",
    #         ),
    #         ComposableNode(
    #             package="joy",
    #             plugin="joy::Joy",
    #             name="joy_node",
    #         ),
    #     ],
    #     output="both",
    # )

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
        output="both",
    )



    return LaunchDescription([
        db_arg,
        ar_model_arg,
        tf_prefix_arg,
        rviz_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_controller_spawner,
        gripper_controller_spawner,
        servo_node,
        robot_state_publisher,
        static_tf_broadcaster,
        moveit_servo,
        joy,
    ])
