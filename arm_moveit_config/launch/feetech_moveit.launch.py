from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    """Main launch entry point for MoveIt demo"""
    package_path = get_package_share_directory("arm_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .joint_limits(package_path + "/config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    kinematics_yaml = PathJoinSubstitution([package_path, "config", "kinematics.yaml"])

    # Launch Arguments
    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Debug mode",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", 
        default_value="true",
        description="Launch RViz",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    publish_robot_description_semantic_arg = DeclareLaunchArgument(
        "publish_robot_description_semantic",
        default_value="true",
        description="Publish semantic robot description",
    )

    db_arg = DeclareLaunchArgument(
        "db",
        default_value="false",
        description="Enable database support",
    )

    # Virtual Joints (conditionally included)
    virtual_joints_launch = include_virtual_joints_launch(package_path)

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_path + "/launch/rsp.launch.py"
        )
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"publish_robot_description_semantic": LaunchConfiguration(
                "publish_robot_description_semantic"
            )},
            {"robot_description_kinematics": kinematics_yaml},
        ],
    )

    # EE Goal Control Node
    # ee_goal_control_node = Node(
    #     package="arm_controller",
    #     executable="ee_goal_control",
    #     name="ee_goal_control",
    #     output="screen",
    #     parameters=[]
    # )

    # RViz
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", get_rviz_config(package_path)],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Warehouse Database
    warehouse_db_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_path + "/launch/warehouse_db.launch.py"
        ),
        condition=IfCondition(LaunchConfiguration("db")),
    )

    # ROS2 Control Node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            package_path + "/config/ros2_controllers.yaml"
        ],
    )

    # Controller Spawners
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_path + "/launch/spawn_controllers.launch.py"
        )
    )

    return LaunchDescription([
        debug_arg,
        use_rviz_arg,
        use_sim_time_arg,
        publish_robot_description_semantic_arg,
        db_arg,
        virtual_joints_launch,
        robot_state_publisher,
        move_group_node,
        # ee_goal_control_node,
        rviz_node,
        warehouse_db_launch,
        ros2_control_node,
        spawn_controllers_launch,
    ])


def include_virtual_joints_launch(package_path):
    """Conditionally include virtual joints if launch file exists"""
    virtual_joints_launch = package_path + "/launch/static_virtual_joint_tfs.launch.py"
    
    if os.path.exists(virtual_joints_launch):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(virtual_joints_launch)
        )
    return []


def get_rviz_config(package_path):
    """Get path to RViz configuration file"""
    return package_path + "/launch/feetech_moveit.rviz"