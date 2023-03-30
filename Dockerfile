FROM ros:noetic-ros-core-focal

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    build-essential \
    python3-catkin-tools \
    python3-rosdep \
    ros-noetic-moveit \
    ros-noetic-moveit-plugins \
    ros-noetic-moveit-planners \
    && rm -rf /var/lib/apt/lists/*

COPY catkin_ws/src /home/catkin_ws/src
COPY catkin_ws/src/simple_scene/worlds /usr/share/gazebo-11/worlds
COPY catkin_ws/src/simple_scene/models /root/.gazebo/models

RUN cd /home/catkin_ws/src \
    && rosdep init \
    && rosdep update

RUN . /opt/ros/noetic/setup.sh \
    && apt-get update \
    && cd /home/catkin_ws \
    && rosdep install --rosdistro=noetic --from-path . -y --ignore-src

RUN . /opt/ros/noetic/setup.sh && cd /home/catkin_ws && catkin build

RUN echo "source /opt/ros/noetic/setup.sh" >> ~/.bashrc