FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-rosdep python3-pytest \
    python3-numpy python3-scipy python3-yaml python3-opencv python3-pyqt5 python3-tk \
    ros-humble-moveit ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-cv-bridge \
    build-essential
WORKDIR /ws
COPY src /ws/src
RUN rosdep update && source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
COPY scripts /ws/scripts
CMD ["bash"]
