# arena-rosnav-3D
This repository combines the 3D ROS simulator Gazebo with Pedsim to provide realistic dyanmic 3D scenarios and tasks to evaluate and and benchmark ROS navigation approaches. It is fully compatible with the planning algorithms trained and developed with arena-rosnav (2D). This presents an essential step in deploying the navigation approaches from arena-rosnav towards real robots. 

The repo currently contains:
* a 3D task generator with 3 modes: random, scenario and manual tasks
* realistic 3D scenarios from AWS
* Integration of the Pedsim-Gazebo pluggin to generate realisitc human behavior 
* Integration of model-based arena-rosnav local planners (needs further testing)
* Partial integration of learning based arena-rosnav planners (RLCA)


## Setup (test)
1. install the arena-rosnav environment:
https://github.com/ignc-research/arena-rosnav
2. Clone this repo into your catkin_ws/src folder: 
```bash
    cd ~/catkin_ws/src
    git clone https://github.com/ignc-research/arena-rosnav-3D.git
```    
3. Run rosws update:
```bash
   cd ~/catkin_ws/src/arena-rosnav-3D
   rosws update --delete-changed-uris .
```
4. Build and source!:
```bash
   cd ~/catkin_ws
   catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
   source devel/setup.zsh
```
## Testing Gazebo simulation and Task Generator
For now only [warehouse world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) is available. To start it, use the supplied launch file and specify the following params: (Bold values -- default)
 * local_planner:=<**teb**, dwa, rlcl, mpc>
 * task_mode:=<**scenario**,random, manual>
 * enable_pedvis:=<**false**, true> if, you want to visualize pedsim agents in rviz
    
    *   Example launch scenario
```bash
    roslaunch task_generator_3d test_task_node.launch local_planner:=dwa task_mode:=random
```
### Random mode
* Use the Generate Task button in Rviz to randomly spawn pedsim agents, robot and set a random goal.
[![random_mode.gif](https://s5.gifyu.com/images/random_mode.gif)](https://gifyu.com/image/GS1K)

### Scenario mode
   * Use the supplied scenario or create your own using [arena-tools](https://github.com/ignc-research/arena-tools). 
   * Scenario files should be placed inside the scenarios folder, remember to change the parameter scenario_file:=<scenario_name.json>, while testing your own scenarios. 
   * In scenario mode all objects will be spawned at their specified position and everything will reset back to this position once the robot reaches it's goal.
   [![scenario_mode.gif](https://s5.gifyu.com/images/scenario_mode.gif)](https://gifyu.com/image/GSAe)
   
## Gazebo
find the project task-description [here](docs/project_tasks.md)
# Arena-Rosnav - 3D
This repository builds on the [arena-rosnav](https://github.com/ignc-research/arena-rosnav) repository and extends its functionalities by including a version of the [pedsim_ros](https://github.com/eliastreis/pedsim_ros) for obstacle management and Gazebo as simulator. (The Flatland-simulator, as used in arena-rosnav is not included). (For a detailed description of differences to arena-rosnav see [here](docs/project_report.md)).

CURRENTLY, THIS PACKAGE IS STILL UNDER DEVELOPMENT. IT IS NOT YET FULLY FUNCTIONAL.

## Examples
| <img width="400" height="400" src="/img/small_warehouse.png"> | <img width="400" height="400" src="/img/outside.png"> |
|:--:| :--:| 
| *Pre-build map* | *Random map* |


## What is this repository for?
Testing DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic 3D environments. Test state of the art local and global planners in ROS environments both in simulation and on real hardware. Following features are included:

* multiple detailed scenario-worlds 
* creation of random 3D-words with static and dynamic obstacles
* different robot models
* realistic behavior patterns and semantic states of dynamic obstacles (by including pedsim's extended social force model)
* Implementation of intermediate planner classes to combine local DRL planner with global map-based planning of ROS Navigation stack
* Testing a variety of planners (learning based and model based) within specific scenarios in test mode
* Modular structure for extension of new functionalities and approaches


## 1. Installation
Please refer to [Installation.md](docs/Installation.md) for detailed explanations about the installation process.

## 2. Usage

### Testing

Please refer to [Testing.md](docs/Testing.md) for detailed explanations about agent, policy and training setups.

**Sample usage**

After successful installation run the following command with your python-env activated (`workon rosnav`).
```bash
roslaunch arena_bringup start_arena_gazebo.launch
```
The following output can be expected:

<img width="1000" height="400" src="/img/aws_house.png">


### Training
***Arena-Rosnav's training functionality is not yet included***

## Miscellaneous

- [How to include further world files](docs/Miscellaneous.md#How-to-include-further-world-files)
- [How to create more world files](docs/Miscellaneous.md#How-to-create-more-world-files)
- [How to speed-up gazebo simulation speed](docs/Miscellaneous.md#How-to-speed-up-gazebo-simulation-speed)
- [How to include further scenarios](docs/Miscellaneous.md#How-to-include-further-scenarios)
- [Further improvement ideas](docs/project_report.md#Open-topics)
- [Detailed description of changes to arena-rosnav](docs/project_report.md) (currently in progress)


# Used third party repos:

* ROS navigation stack: http://wiki.ros.org/navigation
* Pedsim: https://github.com/srl-freiburg/pedsim_ros
* Small-warehouse world: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
* Small-house world: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
* Turtlebot3-robot & house-world: https://github.com/ROBOTIS-GIT/turtlebot3_simulations

