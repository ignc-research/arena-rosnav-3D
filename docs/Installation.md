## 1. Installation
#### 1.1. Standard ROS setup
(Code has been tested with ROS-melodic on Ubuntu 18.04 and Python 3.6)

* Configure your Ubuntu repositories
```
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt update
```

* Setup your scources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

*	Set up your keys
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

*	Installation
```
sudo apt update
sudo apt install ros-melodic-desktop-full
```

* Environment Setup
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

*	Dependencies for building packages
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

* Initialize rosdep
```
sudo rosdep init
rosdep update
```

* Install additional pkgs 
```
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
```

#### 1.2. Prepare virtual environment & install python packages
To be able to use python3 with ROS, you need an virtual environment. We recommend using virtualenv & virtualenvwrapper. 

* Install virtual environment and wrapper (as root or admin! with sudo) on your local pc (without conda activated. Deactivate conda env, if you have one active)
```
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv
sudo pip3 install virtualenvwrapper
which virtualenv   # should output /usr/local/bin/virtualenv  
```

* Create venv folder inside your home directory
```
cd $HOME
mkdir python_env   # create a venv folder in your home directory 
```

* Add exports into your .bashrc:
```
echo "export WORKON_HOME=$HOME/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3 
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
```

* Create a new venv

Note: You might need to restart your terminal at this point.
```
mkvirtualenv --python=python3.6 rosnav
workon rosnav
```

* Install packages inside your venv (venv always activated!):
```
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip install pyyaml catkin_pkg netifaces pathlib
```     

* Install stable_baselines3 for training DRL into your venv (venv always activated!)
```
pip install stable-baselines3
```

### 1.3 setup gazebo_ros for python3 compatibility
To ensure python3 compatibility the `gazebo_ros` package must be changed: (we recomend the following steps):
1. run this command in the terminal to receive writing rights for the package:
```bash
sudo chown -R $USER:$USER /opt/ros
```
go to `/opt/ros/melodic/lib/gazebo_ros` and change *#!/usr/bin/env python2.7*  to  *#!/usr/bin/env python*

#### 1.4. Install arena-rosnav repo
* Create a catkin_ws and clone this repo into your catkin_ws 
````
cd $HOME
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/eliastreis/arena-rosnav-3D.git

cd arena-rosnav-3D && rosws update --delete-changed-uris .
source $HOME/.bashrc
cd ../.. 
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
````
Note: if you use zsh replace bash with zsh in the commands

  * Install ros geometry and geometry2 from source (compiled with python3) 

The official ros only support tf2 with python2. In order to make the *tf* work in python3, its necessary to compile it with python3. Run the following commands in the terminal:
```
mkdir -p ~/rosws/src
cd rosws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
echo "source ~/rosws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd ~/rosws/src
git clone https://github.com/ros/geometry2.git -b melodic-devel
git clone https://github.com/ros/geometry.git -b melodic-devel
cd .. && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

* Set python path in .bashrc (or .bashrc if you use that)
```
nano ~/.bashrc
```
Add these lines below "source/opt/ros/melodic/setup.bash"
```
source /$HOME/catkin_ws/devel/setup.bash
export PYTHONPATH=$HOME/catkin_ws/src/arena-rosnav-3D:${PYTHONPATH}
```
Add this line above "source/opt/ros/melodic/setup.bash"
```
export PYTHONPATH=""
```

* Inside forks/stable-baselines3
```
pip install -e .

```
* inside catkin_ws:
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```


* To install all dependent packages for arena-rosnav use `rosdep`
```bash
sudo apt install python-rosdep
sudo rosdep init
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

* To install pedsim (for obstacle management) run the following command: 
```bash
cd ~/catkin_ws/src/arena-rosnav-3D
git clone https://github.com/eliastreis/pedsim_ros.git
```
* To use the cadrl planer install the following ternsorflow version:
```bash
pip install tensorflow==1.15
```  



# Error Handling 
if you encounter the error "world path not given", it is probably because you havent updated the forks repository or working on an old branch.
In that case go to the arena-rosnav folder and do
```
rosws update
```
Subsequently, go to the forks/stable_baselines3 folder and do:
```
pip install -e .
```

# Training with GPU RTX 3090
in order to train with an NVIDIA GPU RTX3090 you need the latest version of pytorch. Inside your venv, do:
```
pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
```
