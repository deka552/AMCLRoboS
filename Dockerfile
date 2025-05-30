FROM ros:humble

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO humble


# ................................................................................................
# Install necessary dependencies .................................................................

# ROS2

RUN apt-get update && apt-get install -y --fix-missing \
  python3 \
  python3-pip \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

  
# Gazebo

# Turtlebot3 and Nav2 from source

RUN apt-get update && apt-get install -y --fix-missing \
  ros-$ROS_DISTRO-test-msgs \
  ros-$ROS_DISTRO-bondcpp \
  ros-$ROS_DISTRO-behaviortree-cpp-v3 \
  ros-$ROS_DISTRO-diagnostic-updater \
  ros-$ROS_DISTRO-rviz-common \
  ros-$ROS_DISTRO-rviz-default-plugins \
  libgraphicsmagick++-dev \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --fix-missing \
  xtensor-dev \
  libceres-dev \
  libompl-dev \
  && rm -rf /var/lib/apt/lists/*


# Cartographer

RUN apt-get update && apt-get install -y --fix-missing \
  ros-$ROS_DISTRO-cartographer \
  ros-$ROS_DISTRO-cartographer-ros \
  && rm -rf /var/lib/apt/lists/*


# RViz

RUN apt-get update && apt-get install -y --fix-missing \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-nav2-bringup \
  && rm -rf /var/lib/apt/lists/*


# ................................................................................................
# cyclonedds .....................................................................................

RUN apt update && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY ./cyclonedds.xml /cyclonedds.xml
ENV CYCLONEDDS_URI=/cyclonedds.xml

# ................................................................................................
# copy pkgs ......................................................................................

RUN apt-get update && apt-get install -y --fix-missing build-essential cmake pkg-config \
  libboost-all-dev libeigen3-dev libprotobuf-dev protobuf-compiler \
  libqt5core5a libqt5gui5 libqt5widgets5 qtbase5-dev \
  libtinyxml-dev libtinyxml2-dev libogre-1.9-dev \
  libogre-1.9.0v5 libsdformat9-dev
  
RUN apt-get update && apt-get install -y --fix-missing \ 
  libqwt-qt5-dev libfreeimage-dev protobuf-compiler libprotobuf-dev \
  libtar-dev
  

RUN git clone https://github.com/osrf/sdformat.git -b sdf9

RUN apt-get install -y  ruby

RUN cd sdformat && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j$(nproc) &&  make install
  
RUN cd ..

# ....................................................................................
    
# RUN git clone https://github.com/osrf/gazebo.git -b gazebo11

# RUN apt-get update && apt-get install -y --fix-missing \
#   libignition-msgs5-dev \
#   libignition-transport8-dev \
#   libignition-common3-dev \
#   libignition-fuel-tools4-dev \
#   && rm -rf /var/lib/apt/lists/*

# RUN cd gazebo && mkdir build && cd build && cmake .. && make -j$(nproc) &&  make install


# ....................................................................................
RUN apt-get update && apt-get install -y --fix-missing \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  && rm -rf /var/lib/apt/lists/*
  
# ....................................................................................

WORKDIR /ros2ws/src
# RUN git clone --branch ros2 https://github.com/ros-simulation/gazebo_ros_pkgs.git
# RUN git clone --branch humble https://github.com/ros-perception/image_common.git

# ....................................................................................

RUN pip3 install pyserial
RUN pip3 install matplotlib

# ................................................................................................
# ls lidar .......................................................................................

RUN apt-get update && apt-get install -y --fix-missing \
  libpcap-dev \
  ros-humble-diagnostic-updater \
  && rm -rf /var/lib/apt/lists/* 

# ....................................................................................
# ....................................................................................

COPY ./turtlebot3/DynamixelSDK /ros2ws/src/turtlebot3/DynamixelSDK
COPY ./turtlebot3/turtlebot3 /ros2ws/src/turtlebot3/turtlebot3
COPY ./turtlebot3/turtlebot3_msgs /ros2ws/src/turtlebot3/turtlebot3_msgs
# COPY ./turtlebot3/turtlebot3_simulations /ros2ws/src/turtlebot3/turtlebot3_simulations
# COPY ./Lslidar_ROS2_driver /ros2ws/src/Lslidar_ROS2_driver
# COPY ./navigation2 /ros2ws/src/navigation2
# COPY ./astabot /ros2ws/src/astabot


# ................................................................................................
# build pkgs .....................................................................................

ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=/opt/ros/humble:$CMAKE_PREFIX_PATH

RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /ros2ws/ && colcon build --symlink-install --parallel-workers 2 --cmake-args -Wno-dev'


COPY ./Lslidar_ROS2_driver /ros2ws/src/Lslidar_ROS2_driver
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /ros2ws/ && colcon build --symlink-install'

COPY ./astabot /ros2ws/src/astabot
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /ros2ws/ && colcon build --symlink-install'

# ................................................................................................
# setup ros environment ..........................................................................

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /ros2ws/install/setup.bash" >> ~/.bashrc

# ................................................................................................
# setup entrypoint ...............................................................................

COPY ./ros_entrypoint.sh /
#RUN chmod 755 ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
