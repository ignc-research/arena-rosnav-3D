# 1. Installation

## 1.1. Standard ROS setup

(Code has been tested with ROS-noetic on Ubuntu 20.04 and Python 3.8)

- Configure your Ubuntu repositories

```
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt update
```

- Setup your scources.list

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

- Set up your keys

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

- Installation

```
sudo apt update
sudo apt install ros-noetic-desktop-full
```

- Environment Setup

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 1.2. Prepare virtual environment & install python packages

- Initialize rosdep

```
sudo apt-get update && sudo apt-get install -y \
libopencv-dev \
liblua5.2-dev \
screen \
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
```

To be able to use python3 with ROS, you need an virtual environment. We recommend using virtualenv & virtualenvwrapper.

- Install virtual environment and wrapper (as root or admin! with sudo) on your local pc (without conda activated. Deactivate conda env, if you have one active)

```
sudo apt install pip
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv
sudo pip3 install virtualenvwrapper
which virtualenv   # should output /usr/local/bin/virtualenv
```

- Create venv folder inside your home directory

```
cd $HOME && mkdir python_env   # create a venv folder in your home directory
```

- Add exports into your .bashrc or .zshrc:

```
echo "export WORKON_HOME=$HOME/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
```

- Create a new venv

Note: You might need to restart your terminal at this point.

```
mkvirtualenv --python=python3.8 rosnav
workon rosnav
```

- Install packages inside your venv (venv always activated!):

```
pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip3 install pyyaml catkin_pkg netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml


```

## 1.3 Install arena-rosnav-3D and pedsim repo

- Create a catkin_ws and clone this repo with its dependencies into your catkin_ws

```
sudo apt-get update
cd $HOME && mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/eliastreis/arena-rosnav-3D.git -b training-actors
cd arena-rosnav-3D && rosws update
cd .. && cd forks/stable-baselines3 && pip install -e .
cd ../../.. && catkin_make
source devel/setup.bash
```

- Add the changes to your .bashrc file
  > Note: if you use zsh replace bash with zsh in the commands or vice versa

```
echo "source $HOME/catkin_ws/devel/setup.bash
export PYTHONPATH=$HOME/catkin_ws/src/arena-rosnav-3D:${PYTHONPATH}" >> ~/.bashrc
```

## 1.3 Include the actor-collsion pluging

This makes the actor model in gazebo visible for the laser scan

```bash
cd && git clone https://github.com/eliastreis/ActorCollisionsPlugin.git
cd ActorCollisionsPlugin && mkdir build && cd build && cmake .. && make
echo "export GAZEBO_PLUGIN_PATH=/home/ActorCollisionsPlugin/build" >> ~/.bashrc
```

# Add arena-rosnav next to arena-rosnav-3D

- Create a catkin_ws and clone this repo with its dependencies into your catkin_ws

```bash
sudo apt-get update
cd $HOME && mkdir -p arena_ws/src && cd arena_ws/src
git clone https://github.com/eliastreis/arena-rosnav.git
cd arena-rosnav && rosws update
```

- Add the changes to your .bashrc file
  > Note: to use arena-rosnav next to its 3D counterpart make sure to source only workspace at the time

```
echo "source $HOME/arena_ws/devel/setup.bash
export PYTHONPATH=$HOME/arena_ws/src/arena-rosnav:${PYTHONPATH}" >> ~/.bashrc
```
<details>
  <summary markdown="span">in your `.bashrc` this should look like this: </summary>

```bash
source /home/elias/arena_ws/devel/setup.bash
export PYTHONPATH=/home/elias/arena_ws/src/arena-rosnav:/home/elias/arena_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages

```
</details>




# Training with GPU RTX 3090

**_NOTE: This section has not been tested on arena-rosnav-3D_**

in order to train with an NVIDIA GPU RTX3090 you need the latest version of pytorch. Inside your venv, do:

```
pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
```

# Trouble shooting

- Follow this steps if you encounter the following stable-baseline related error:

        #### Trouble Shooting
        While trying the Quickstart you might encouter the following error in the second terminal:
        ```
        Traceback (most recent call last):
          File "scripts/training/train_agent.py", line 229, in <module>
            treshhold_type="succ", threshold=0.9, verbose=1)
        TypeError: __init__() got an unexpected keyword argument 'treshhold_type'
        ```
        This error can be resolved by updating your stable baselines and your workspace. Therefore run the following commands:
        ```
        cd $HOME/catkin_ws/src/forks/stable-baselines3
        pip install -e.
        ```
        ```
        cd $HOME/catkin_ws/src/arena-rosnav
        rosws update
        ```
        ```
        cd $HOME/catkin_ws
        catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
        ```
