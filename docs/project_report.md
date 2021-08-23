**Table of contents**

- [Introduction](#introduction)
- [Approach](#approach)
- [New Structure](#new-structure)
  - [Arena_bringup](#arena_bringup)
  - [Arena_scenario](#arena_scenario)
  - [Task_generator](#task_generator)
  - [Pedsim_ros](#pedsim_ros)
- [Open topics](#open-topics)
# Introduction
# Approach
# New Structure
## Arena_bringup
## Arena_scenario
## Task_generator
## Pedsim_ros
# Open topics
| **Topic** | **Description**|
|:--:| :--:| 
| Update free space indices | Currently the forbidden zones are not shared between the object generators and the robot_manager. Therefore it is possible, that the robot's navigation goal is also e.g. the start position of a dynamic obstacle |
| Register interactive obstacles | In the outdoor random mode, the randomly created static obstacles are not registered in Pedsim as (interactive) obstacles. The PedsimManager could be extended to include that functionality. |
| Extent scenarios | We currently have only one scenario per world |
| Scenario Editor | Simple scenario creation with [arena-tools](https://github.com/ignc-research/arena-tools) could be integrated to build scenarios (and create a pedsim scenario) |
| Waypoints dynamic obstacles | Extend the obstacle manager to select only targets in wide open spaces to allow smooth object movement. |
| Actor integration | To create moving pedestrians with walking animation see: [here](http://gazebosim.org/tutorials?tut=actor&cat=build_robot) and [here](https://github.com/BruceChanJianLe/gazebo-plugin-autonomous-actor/) |
| Update package.xml & CMakes | Update package.xml and CMakelists. Pot. create a Metapackage |
| Arena_bringup | Some launch files are currently (unnecessarily) overloaded with args and params. These could be simplified and put into a coherent order to increase comprehensibility. |
| More obstacles | We currently support only 1 type of dynamic and 2 types of static obstacles. This could be easily extended  |