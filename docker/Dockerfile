################################################################i
# Dockerfile for Arena-Rosnav-3D

# URL: https://github.com/ignc-research/arena-rosnav-3D

# Based on Ubuntu 20.04 + ROS-Noetic-Desktop-Full Version

#################################################################

# This image includes additional meta-packages such for desktop installations than offical image
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# Install basic linux packages

RUN apt-get -y update && apt-get install -y \
    software-properties-common \
    wget \
    curl \
    apt-utils\
    gnutls-bin \
    vim \
    git \
    original-awk \
    python3-pip \
    screen \
    libopencv-dev \
    liblua5.2-dev \
    && add-apt-repository ppa:rock-core/qt4 \
    && apt-get install -y libqtcore4


RUN apt-get -y update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    build-essential \
    python3-rospkg-modules \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-mpc-local-planner \
    libarmadillo-dev \
    ros-noetic-nlopt \
    ros-noetic-turtlebot3-description \
    ros-noetic-turtlebot3-navigation \
    ros-noetic-lms1xx \
    ros-noetic-velodyne-description \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    python3-empy \
    python3-setuptools \
    libarmadillo-dev \
    ros-noetic-pcl-conversions\
    ros-noetic-pcl-ros\
    ros-noetic-desktop-full\

    && echo $'\n\
    source /opt/ros/noetic/setup.sh' >> /root/.bashrc
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration


# 2. Prepare virtual environment
RUN pip3 install --upgrade pip \
    && pip3 install virtualenv  virtualenvwrapper\
    && cd /root \
    && mkdir .python_env \
    && echo $'\n\
    export WORKON_HOME=/root/.python_env \n\
    export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 \n\
    export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv \n\
    export PYTHONPATH=/root/catkin_ws/src/arena-rosnav-3D:${PYTHONPATH} \n\
    source /usr/local/bin/virtualenvwrapper.sh' >> /root/.bashrc

ENV WORKON_HOME=/root/.python_env \
    VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 \
    VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv

# 3. Create python virtualenv and install dependencies
RUN source /root/.bashrc \
    && source `which virtualenvwrapper.sh` \
    && mkvirtualenv --python=python3.8 rosnav \ 
    && workon rosnav \
    && pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed \
    && pip3 install pyyaml catkin_pkg netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml 
#&& pip3 --no-cache-dir install stable-baselines3


## 4.1. Install base arena-rosnav-3d
RUN . /opt/ros/noetic/setup.sh \
    && source /root/.bashrc \
    && cd $HOME \
    && mkdir -p catkin_ws/src && cd catkin_ws/src \
    && git clone https://github.com/ignc-research/arena-rosnav-3D.git -b arena-benchmark \
    && cd arena-rosnav-3D\
    && rosws update \
    && cd ../.. \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && source devel/setup.bash \
    && tar -xf $HOME/catkin_ws/src/arena-rosnav-3D/simulator_setup/robot/agv-ota/meshes/OTAv07_meshes/OTA-v0.7.tar.xz -C $HOME/catkin_ws/src/arena-rosnav-3D/simulator_setup/robot/agv-ota/meshes/OTAv07_meshes/ \
    && echo $'\n \
    source $HOME/catkin_ws/devel/setup.sh \n\
    export PYTHONPATH=$HOME/catkin_ws/src/arena-rosnav-3D:${PYTHONPATH}' >> ~/.bashrc 

## 4.2. Include the actor-collision plugin
RUN cd $HOME \
    && git clone https://github.com/eliastreis/ActorCollisionsPlugin.git \
    && cd ActorCollisionsPlugin \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && echo \
    "export GAZEBO_PLUGIN_PATH=$HOME/ActorCollisionsPlugin/build " \
    >> ~/.bashrc 

## 4.3. Install Pedsim
RUN . /opt/ros/noetic/setup.sh \
    && sudo apt install python3-rosdep python3-rospkg \
    #&& sudo rosdep init \
    && cd ~/catkin_ws/src/arena-rosnav-3D \
    && rosws update \
    && cd ~/catkin_ws \
    # && cd $HOME/catkin_ws/src/forks/arena-tools \
    # && git clone https://gitlab.com/LIRS_Projects/LIRS-WCT lirs-wct\
    # && . lirs-wct/deploy.sh \
    && rosdep install --from-paths src --ignore-src -r -y \
    && cd ~/catkin_ws/src/forks/pedsim_ros \
    && git submodule update --init --recursive \
    && cd ../../.. && catkin_make --only-pkg-with-deps spencer_tracking_rviz_plugin \
    && catkin_make -DCATKIN_WHITELIST_PACKAGES="" 

COPY /entrypoint_testing.sh /entrypoint_testing.sh
ENTRYPOINT ["/entrypoint_testing.sh"]