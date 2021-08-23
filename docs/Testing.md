# Introduction
Arena-rosnav-3D is structurally very similar to arena-rosnav to allow easy switching between 2D and 3D environments. Currently, three custom worlds and a random generator are supported. In our detailed custom worlds you can generate dynamic obstacles either in a scenario mode or in a random mode where the obstacle trajectories are determined randomly. 

# Worlds
We provide the following four worlds for your use: 
||||||
## Pre-build world
To select one of our pre-build worlds, specify your scenario by selecting **aws_house**,  turtlebot3_house, small_warehouse *(bould is your default world)* in your startup file:
```bash
roslaunch arena_bringup start_arena_gazebo.launch world:=turtlebot3_house
```
Since our pre-build worlds are very detailed we do not spawn further static obstacles. For the dynamic obstacle mode see here. To implement your own custom world file see [here]()
## Random world
To select our custom world generator specify the outside argument with `true` (default is false)
```bash
roslaunch arena_bringup start_arena_gazebo.launch outside:=true
```
The custom world generator creates a map with a set number of static and dynamic obstacles of a randomly determined size (and shape).

If you want to change the number of static obstacles you do this [here]()

If you want to change the number of dynamic obstacles you do this [here]()

# Task-Mode
We currently support a random and a dynamic mode (you can also set the robot goal manually using Rviz).

Select either **random** or scenario and run:
```bash
roslaunch arena_bringup start_arena_gazebo.launch task_mode:=random
```
## Random Mode
In random mode arena-rosnav chooses the robot start and goal position, as well as the trajectory of the dynamic obstacles randomly. 

If you want to change the number of dynamic obstacles you do this [here]().

Find the associated code [here]().
## Scenario Mode
In scenario mode, object obstacle and robot behavior is scripted. We support currently one scenario per world. To extend this and build your own custom scenarios see [here]().

Find the associated code [here]().

# Local Planer
local_planner ToDo

# Advanced Parameters
You can further customize the simulation altering the following parameters:

| Name | Options | Description |
|:--:| :--:| :--:|  
| train_mode | true, **false** |  Since training mode is not yet implemented `false` should stay selected. (This would deactivate e.g. the task_generator) |
| disable_scenario | true, **false** | This parameter would e.g. disable the task generator and other selected notes. This should only be changed with caution |
| reset | **true**, false | Determines whether the scenario should be reset (or terminated) after reaching its objective (Robot has reached navigation goal) |
| enable_pedsim | **true**, false | Pedsim is used to for dynamic obstacle management. Setting this to false should disable dynmaic obstacles |