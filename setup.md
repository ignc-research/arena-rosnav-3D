## Ros-melodic setup
To install ros melodic on python 3 take the following steps:
1. (If you had some previous ros-versions installed download, run the following commands to uninstall them:)
```bash
sudo apt-get remove ros-*
sudo apt-get remove ros-melodic-*
sudo apt-get autoremove
```
2.Download & run the following script to install ros melodic (incl virtual env). Before running replace <code>PATH</code> with the path to the file
```bash
PATH/install_ros1.sh 
```
## Installation Gazebo
To install gazbo (incl. dependencies) download the `install_gazebo.sh` file.  Run the code below in the comandline. Before replace <code>PATH</code> with the path to the file
```bash
PATH/install_gazebo.sh 
```
To verify a successful installation, run the command <code>gazebo</code> in the terminal. The Gazebo simulator should start.

---
