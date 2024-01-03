sudo apt update -q && sudo apt install -y \
  iputils-ping\
  iproute2 \
  libasio-dev \
  libpcap-dev \
  python3-rosdep \
  snapd \
  git \
  tmux \
  vim \
  nano \
  wget \
  espeak \
  git \
  libusb-1.0-0-dev \
  zsh \
  libqt5serialport5-dev \
  software-properties-common \
  gnome-terminal \
  dbus-x11 \
  libcanberra-gtk-module \
  libcanberra-gtk3-module \
  usbutils \
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
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl


python3 -m pip install --upgrade pip
python3 -m pip install transforms3d websocket-client catmux distro numpy opencv-python depthai numpy pyttsx3 

git cd $HOME && \
    clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd ./Livox-SDK2 && \
    mkdir build  && \
    cd build && \
    cmake .. && make -j && \
    make install && \
    cd ../.. && \
    rm -rf Livox-SDK2