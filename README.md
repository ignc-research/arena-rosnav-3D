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
- [How to include further scenarios](docs/Miscellaneous.md#How-to-include-further-scenarios)
- [Further improvement ideas](docs/project_report.md#Open-topics)
- [Detailed description of changes to arena-rosnav](docs/project_report.md) (currently in progress)


# Used third party repos:

* ROS navigation stack: http://wiki.ros.org/navigation
* Pedsim: https://github.com/srl-freiburg/pedsim_ros
* Small-warehouse world: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
* Small-house world: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
* Turtlebot3-robot & house-world: https://github.com/ROBOTIS-GIT/turtlebot3_simulations

