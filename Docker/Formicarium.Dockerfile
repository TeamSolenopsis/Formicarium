# Use the ROS2 base image
FROM osrf/ros:humble-desktop-full-jammy

# Install any additional dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip

RUN pip install pygame

# Clone your ROS2 package from GitHub
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/src

# #change the link to the github repo
RUN git clone https://github.com/TeamSolenopsis/Formicarium.git
RUN pip install setuptools==58.2.0
WORKDIR /ros_ws