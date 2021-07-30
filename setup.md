## Ros-melodic setup
To install ros melodic on python 3 take the following steps:

1. (If you had some previous ros-versions installed download, run the following commands to uninstall them:)
```bash
sudo apt-get remove ros-*
sudo apt-get remove ros-melodic-*
sudo apt-get autoremove
```
\
2. Download & run the following script to install ros melodic (incl virtual env). Before running replace <code>PATH</code> with the path to the file
```bash
PATH/install_ros1.sh 
```
\
3. Restart the terminal and load the `install_ros2` module (with adjusted PATH) in the terminal.
```bash
PATH/install_ros1.sh 
```
\
4.  Set python path in .zshrc (or .bashrc if you use that)
```
nano ~/.zshrc
```
Add these lines below "source/opt/ros/melodic/setup.zsh"
```
source /$HOME/catkin_ws/devel/setup.zsh
export PYTHONPATH=$HOME/catkin_ws/src/arena-rosnav:${PYTHONPATH}
export PYTHONPATH=$HOME/geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
```
Add this line above "source/opt/ros/melodic/setup.zsh"
```
export PYTHONPATH=""
```
\
5. To install final packages run (with adjusted PATH variable):
```bash
PATH/install_ros1.sh 
```

---
## Installation Gazebo
To install gazbo (incl. dependencies) download the `install_gazebo.sh` file.  Run the code below in the comandline. Before replace <code>PATH</code> with the path to the file
```bash
PATH/install_gazebo.sh 
```
To verify a successful installation, run the command <code>gazebo</code> in the terminal. The Gazebo simulator should start.

---
