from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("urdf_6dof_arm", package_name="moveit_6dof_arm")
        .joint_limits(
            get_package_share_directory("moveit_6dof_arm")
            + "/config/joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    # Static TF publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using mock hardware for trajectory execution
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_6dof_arm"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        ros2_control_node,
    ] + generate_demo_launch(moveit_config).entities)
