# 6-DOF Arm Configuration

This repository contains the configuration files for a 6DOF robotic arm, including URDF, SDF, and launch files for simulation and visualization in ROS2.

For integration with actual hardware and more detailed information, please refer to the [main repository](https://github.com/iltlo/feetech_arm_control.git).

## Installation

1. Clone the repository into your ROS2 workspace:
    ```bash
    cd ~/your_ros2_ws/src
    git clone https://github.com/iltlo/config_6dof_arm.git
    ```

2. Build and source the workspace:
    ```bash
    cd ~/your_ros2_ws
    colcon build
    source ~/your_ros2_ws/install/setup.bash
    ```

## Usage

To run the MoveIt! demo:
```bash
ros2 launch moveit_6dof_arm demo.launch.py
```

To display the robot model in RViz:
```bash
ros2 launch urdf_6dof_arm display.launch.py
```
