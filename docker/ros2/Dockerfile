FROM ghcr.io/tiryoh/ros2-desktop-vnc:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      curl \
      gdb \
      git \
      iputils-ping \
      net-tools \
      ros-humble-moveit \
      ros-humble-rviz2 \
      ros-humble-ur && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/ubuntu/ur_ros2_ws/src && \
    chown -R 1000:1000 /home/ubuntu/ur_ros2_ws && \
    echo 'source /opt/ros/humble/setup.bash' >> /home/ubuntu/.bashrc && \
    echo 'export ROS_DOMAIN_ID=42' >> /home/ubuntu/.bashrc
