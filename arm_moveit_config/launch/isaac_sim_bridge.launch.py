from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Create the MoveIt configuration
    moveit_config = MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config").to_moveit_configs()

    # Include the demo launch file
    demo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("arm_moveit_config"),
            "launch",
            "demo.launch.py"
        ])
    )

    # Create a joint state publisher for simulation
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "rate": 50,  # Publishing rate in Hz
            "source_list": ["joint_states"],
            "use_gui": False
        }]
    )

    # Create a robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": moveit_config.robot_description,
            "publish_frequency": 50.0
        }]
    )

    # Return the launch description
    return LaunchDescription([
        demo_launch,
        joint_state_publisher,
        robot_state_publisher
    ])
