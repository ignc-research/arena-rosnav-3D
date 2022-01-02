################################################################i
# Dockerfile for Arena-Rosnav-3D

# URL: https://github.com/ignc-research/arena-rosnav-3D

# Based on Ubuntu 20.04 + ROS-Noetic-Desktop-Full Version

#################################################################

# This image includes additional meta-packages such for desktop installations than offical image
FROM osrf/ros:noetic-desktop-full


# Set up Oh-my-zsh
RUN apt-get -y update && apt-get install -y \
    curl \
    wget \

RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t https://github.com/denysdovhan/spaceship-prompt \
    -a 'SPACESHIP_PROMPT_ADD_NEWLINE="false"' \
    -a 'SPACESHIP_PROMPT_SEPARATE_LINE="false"' \
    -p git \
    -p ssh-agent \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions


# 1. Install additional pkgs
RUN apt-get -y update && apt-get install -y \
    gnutls-bin \
    vim \
    git \
    original-awk \
    python3-pip \
    libqt4-dev \
    screen \
    libopencv-dev \
    liblua5.2-dev \
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
&& echo $'\n\
source /opt/ros/noetic/setup.zsh' >> /root/.zshrc
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
&& DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration
RUN apt-get install -y software-properties-common



# 2. Prepare virtual environment
RUN pip3 install --upgrade pip \
&& pip3 install virtualenv  virtualenvwrapper\
&& cd /root \
&& mkdir .python_env \
&& echo $'\n\
export WORKON_HOME=/root/.python_env \n\
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 \n\
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv \n\
export PYTHONPATH=/root/catkin_ws/src/arena-rosnav:${PYTHONPATH} \n\
source /usr/local/bin/virtualenvwrapper.sh' >> /root/.zshrc

ENV WORKON_HOME=/root/.python_env \
    VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 \
    VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
    
# 3. Create python virtualenv and install dependencies
RUN . /usr/local/bin/virtualenvwrapper.sh \
&& mkvirtualenv --python=python3.8 rosnav \ 
&& workon rosnav \
&& pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed \
&& pip3 install pyyaml catkin_pkg netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml \
#&& pip3 --no-cache-dir install stable-baselines3

# 4. Install Arena-Rosnav repo and compile

## 4.1. Install base arena-rosnav-3d
RUN mkdir -p catkin_ws/src && cd catkin_ws/src \
&& git clone https://github.com/ignc-research/arena-rosnav-3D.git \
&& cd .. \
&& catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
&& source devel/setup.zsh \
&& echo \
"source $HOME/catkin_ws/devel/setup.zsh export PYTHONPATH=$HOME/catkin_ws/src/arena-rosnav-3D:${PYTHONPATH}" \
>> ~/.zshrc \

## 4.2. Include the actor-collision plugin
RUN cd $HOME \
&& git clone https://github.com/eliastreis/ActorCollisionsPlugin.git \
&& cd ActorCollisionsPlugin \
&& mkdir build \
&& cd build \
&& cmake .. \
make \
&& echo \
"export GAZEBO_PLUGIN_PATH=$HOME/ActorCollisionsPlugin/build " \
>> ~/.bashrc \

## 4.3. Install Pedsim
RUN sudo apt install python-rosdep python-rospkg \
&& sudo rosdep init \
&& cd ~/catkin_ws/src/arena-rosnav-3D \
&& rosws update \
&& cd ~/catkin_ws \
&& rosdep install --from-paths src --ignore-src -r -y \
&& cd ~/catkin_ws/src/forks/pedsim_ros \
&& git submodule update --init --recursive \
&& cd ../../.. && catkin_make --only-pkg-with-deps spencer_tracking_rviz_plugin \
&& catkin_make -DCATKIN_WHITELIST_PACKAGES="" \


# 7. Entrypoint
# RUN ["chmod", "+x", "/root/catkin_ws/src/arena-rosnav/docker/entrypoint_train.sh"]
# WORKDIR /root/catkin_ws/src/arena-rosnav/
# ARG ROS_START="train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4"
# ARG PYTHON_START="--custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn relu"
# ENV roslaunchParams="$ROS_START"
# ENV pythonParams="$PYTHON_START"
# ENTRYPOINT docker/entrypoint_train.sh "$roslaunchParams" "$pythonParams"
