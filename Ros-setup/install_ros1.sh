#!/usr/bin/env bash

sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update

#additional packages
sudo apt-get update && sudo apt-get install -y \
libqt4-dev \
libopencv-dev \
liblua5.2-dev \
screen \
python3.6 \
python3.6-dev \
libpython3.6-dev \
python3-catkin-pkg-modules \
python3-rospkg-modules \
python3-empy \
python3-setuptools \
ros-melodic-navigation \
ros-melodic-teb-local-planner \
ros-melodic-mpc-local-planner \
libarmadillo-dev \
ros-melodic-nlopt \

#create the virtual env
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv
sudo pip3 install virtualenvwrapper
cd $HOME
mkdir python_env 
echo "export WORKON_HOME=$HOME/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3 
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.zshrc
source $HOME/.zshrc
mkvirtualenv --python=python3.6 rosnav
workon rosnav
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip install pyyaml catkin_pkg netifaces pathlib
pip install stable-baselines3

#arena-rosnav related downloads
cd $HOME
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/ignc-research/arena-rosnav
cd arena-rosnav && rosws update
source $HOME/.zshrc
cd $HOME/catkin_ws/src/
git clone https://github.com/eliastreis/arena-rosnav-3D.git
cd ../.. 
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
cd $HOME/catkin_ws/src/arena-rosnav
./geometry2_install.sh
cd $HOME/geometry2_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3



