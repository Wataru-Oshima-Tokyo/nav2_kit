sudo apt update -q && apt install -y \
  net-tools \
  iputils-ping \
  iproute2 \
  libasio-dev \
  libpcap-dev \
  git \
  nano \
  vim \
  python3-rosdep \
  tmux \
  python3-pip \
  gazebo \
  iproute2 \
  libasio-dev \
  libpcap-dev \
  libqt5serialport5-dev \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard \
  ros-humble-laser-proc \
  ros-humble-ros-ign-bridge \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3 \
  ros-humble-rqt* \
  ros-humble-controller-interface \
  ros-humble-effort-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-robot-localization \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-pcl-conversions \
  ros-humble-twist-mux \
  ros-humble-xacro \
  ros-humble-librealsense2* \
  ros-humble-nav2* \
  ros-humble-realsense* \
  ros-humble-apriltag-ros \
  ros-humble-rmw-cyclonedds-cpp 

# Pip installs
python3 -m  pip install  \
  websocket-client catmux numpy distro pyyaml
