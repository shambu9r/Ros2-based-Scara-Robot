# Ros2-based-Scara-Robot

The [ros2_control](https://control.ros.org) framework is a realtime control framework designed for general robotics applications that gains more and more attention both for research and industrial purposes. An overview of the ros2_control framework can be found [here](resources/r2c_overview.md).

This tutorial is made to understand the basic concepts of controlling a robot using ros2_control. In particular, it describes how to :
- [Write a URDF description of a simple SCARA manipulator](resources/urdf_tutorial.md)
- [Launch and interact with the SCARA robot](resources/launch_tutorial.md) 
- [Write a custom hardware interface for the SCARA robot](resources/hardware_tutorial.md)
- [Write a custom controller for the SCARA robot](resources/controller_tutorial.md)
- [Set up the SCARA manipulator to run with ros2_control and Gazebo](resources/gazebo_tutorial.md)

 [SCARA KINEMATICS](scara_robot/src)


![scara model](resources/scara_model.png)

## Getting Started
***Required setup : Ubuntu 22.04 LTS and ROS2 Humble***

1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
