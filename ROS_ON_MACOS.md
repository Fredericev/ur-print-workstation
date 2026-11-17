# Practical ROS 2 options on macOS

macOS can build ROS 2 core packages from source, but this project still
recommends Docker / Linux for ROS 2 and the UR driver.

There are three reasons:

1. Official ROS 2 Humble binary packages do not include macOS. The official Humble installation documentation lists Ubuntu 22.04, RHEL 8, and Windows 10 as binary platforms; macOS follows the source-build route.
2. ROS recommends that new users prefer a Tier 1 operating system for their ROS distribution. Non-Tier 1 systems may require more debugging, and a virtual machine or Docker is also recommended.
3. The official Universal Robots ROS2 Driver states that Linux is a baseline requirement; other operating systems are currently unsupported.

For this project:

- Building from source on macOS is an option for learning ROS 2 nodes, topics, and services.
- For controlling a real UR3e, use Docker, an Ubuntu VM, or a separate Ubuntu computer.
- This project currently uses Docker so that a macOS computer can run the Linux versions of `ur_robot_driver`, MoveIt, and RViz.

References:

- [ROS 2 Humble installation](https://docs.ros.org/en/humble/Installation.html)
- [ROS getting started notes](https://www.ros.org/blog/getting-started/)
- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
