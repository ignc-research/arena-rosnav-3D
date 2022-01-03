# Introduction
Arena-rosnav-3D is structurally very similar to arena-rosnav to allow easy switching between 2D and 3D environments. Currently, three custom worlds and a random world generator are supported. In our detailed custom worlds you can generate dynamic obstacles either in a scenario mode or in a random mode where the obstacle trajectories are determined randomly. 

# Worlds
We provide the following worlds for your use: 
|<img width="150" height="140" src="/img/aws_house-2.png">|<img width="150" height="140" src="/img/turtlebot3_house.png">|<img width="150" height="140" src="/img/small_warehouse-2.png">|<img width="150" height="140" src="/img/outside.png">|<img width="150" height="140" src="/img/factory.png">|<img width="150" height="140" src="/img/hospital.jpg">|
|:--:       |              :--:|             :--:|          :--:|             :--:|          :--:|
| *aws_house* | *turtlebot3_house* | *small_warehouse* | *random world* | *factory* | *hospital* | 

|<img width="150" height="140" src="/img/experiment_rooms.jpg">|<img width="150" height="140" src="/img/bookstore.jpeg">|<img width="150" height="140" src="/img/tb_world.jpg">|
|:--:|        :--:|      :--:|     
| *experiment_rooms* | *bookstore* | *turtlebot3_world* |

Furthermore, an outside world is available using the flag "outside:=true", e.g.
```
roslaunch arena_bringup start_arena_gazebo.launch outside:=true
```

## Pre-build world

To select one of our pre-build worlds, specify your scenario by selecting **aws_house**,  turtlebot3_house, small_warehouse *(bold is your default world)* in your startup file:
```bash
roslaunch arena_bringup start_arena_gazebo.launch world:=turtlebot3_house
```
Since our pre-build worlds are very detailed we do not spawn further static obstacles. For obstacle and robot task-management see [here](#task-mode). 

To implement your own custom world file see [here](/docs/Miscellaneous.md#How-to-include-further-world-files)

## Random world
To select our custom world generator specify the outside argument with `true` (default is false)
```bash
roslaunch arena_bringup start_arena_gazebo.launch outside:=true
```
The custom world generator creates a map with a set number of static and dynamic obstacles of a randomly determined size (and shape).

If you want to change the number of static obstacles you can also do that via the terminal (the shape is randomly determined by the task-generator node):
```bash
roslaunch arena_bringup start_arena_gazebo.launch outside:=true num_static_obs:=10
```

If you want to change the number of actors you can specify the amount using parameters like this:

```bash
roslaunch arena_bringup start_arena_gazebo.launch outside:=true agents:=4
```
## World generation with arena-tools and LIRS
By combining the random 2d map generation feature from our own **arena-tools** with the seamless image to Gazebo world conversion of **LIRS_World_Construction_Tools** we can test our navigation approaches on magnitude of 3D worlds, varying in layout, complexity.
For more information on how to use this feature please refer to [arena-tools](https://github.com/Jacenty00/arena-tools). Otherwise, if you already have your own map image in mind, visit [LIRS_World_Construction_Tools](https://gitlab.com/LIRS_Projects/LIRS-WCT) to gain information on how to convert it into a Gazebo world.

---
# Task-Mode
We currently support a random, scenario or manual task mode.

Select either **random** or scenario, manual and run:
```bash
roslaunch arena_bringup start_arena_gazebo.launch task_mode:=random
```
## Random Mode
In random mode arena-rosnav chooses the robot start and goal position, as well as the trajectory of the dynamic obstacles randomly. 

If you want to change the number of dynamic obstacles, use the following parameter:

```bash
roslaunch arena_bringup start_arena_gazebo.launch task_mode:=random agents:=9
```

## Scenario Mode
In scenario mode, the robot's starting/end position and the amount of actors, as well as their trajectories are set beforehand. We support currently one scenario per world. To extend this and build your own custom scenarios see [here](/docs/Miscellaneous.md#How-to-include-further-scenarios).

```bash
roslaunch arena_bringup start_arena_gazebo.launch task_mode:=scenario
```
If you want to select your custom scenario, run:
```bash
roslaunch arena_bringup start_arena_gazebo.launch task_mode:=scenario scenario_file:={Name of your scenario}
```

---
# Local Planners 
We offer the following local planners [teb, dwa, mpc, rlca, cadrl, arena], which can be used by setting the local planner argument like this:

```bash
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=teb
```

# Robots
We support four different robots. 
|<img width="250" height="240" src="/img/robots/turtlebot3-burger.jpg">|<img width="250" height="240" src="/img/robots/jackal.jpg">|<img width="250" height="240" src="/img/robots/ridgeback.jpg">|<img width="250" height="240" src="/img/robots/agv-ota.png">|
|:--:       |              :--:|             :--:|          :--:| 
| *turtlebot3-burger* | *jackal* | *ridgeback* | *agv-ota* |



All robots are equipped with a laser scanner. The robots differ in size, laser-range etc. See below table for more detailed information on each robot:


| Name  | Max Speed (v_x) [_m/s_]  | Max Speed (v_y) [_m/s_]  | Max Rotational Speed (θ_y) [_rad/s_]  | Radius [_m_] | Emergency-Stop¹ | Laser-range [_m_] | Holonomic² |
| :--- | :---:|  :---: |:---: |:---: |:---:|   :---:| :---:| 
| *turtlebot3-burger* | 0.22 | 0.0  | 2.84  | 0.113 | False | 3.5  | False |
| *jackal*            | 2.0  | 0.0  | 4.0  | 0.267 | False | 30.0 | False |
| *ridgeback*         | 1.1  | 0.5  | 2.0  | 0.625 | False | 10.0 | True  |
| *agv-ota*           | 0.5  | 0.0  | 0.4  | 0.629 | False | 5.0  | False |


For additional / more detailed information about each robot:

+ [See the parameters needed for the **Navigation stack**](https://github.com/eliastreis/arena-rosnav-3D/tree/main/arena_navigation/arena_local_planer/model_based/conventional/config)
+ [See additional robot parameters like laser min/max [_rad_]](https://github.com/eliastreis/arena-rosnav-3D/tree/main/arena_bringup/launch/sublaunch_testing/robot_params)
+ See [_HERE_](https://github.com/eliastreis/arena-rosnav-3D/tree/main/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/configs) for the definition of the robots action_spaces (needed for rl-based-training)

> ___NOTE___: The _emergency-stop_ capability is currently still being development, it will however be available on all robots.

To select a robot model for your simulation run (in this case _ridgeback_):
```bash
roslaunch arena_bringup start_arena_gazebo.launch model:=ridgeback
```
¹ *Stops moving when an object has been detected in the safety zone of the robot*

² *For _holonomic_ robots `vel_y = 0`; they are not able to drive directly to their left or right, but have to drive on a circular trajectory to their reach navigation goal*   
# Advanced Parameters
You can further customize the simulation altering the following parameters:

| Name  | Options   | Description   |
|:--:   | :--:      | :--:          |  
| actors | Integer | Specify the amount of actors to be inserted into a Gazebo world. All of the actors position are then managed by the pedsim simulator |
| actor_height | Double | A fixed actor height on a per world basis needs to be set, default value of 1.1 is already set and should be appropriate for most of the worlds, if not experimentation for proper value will be needed |
| additional_map | true, **false** | Set to true, if you wish to use an additional, more detailed occupancy map for obstacles/pedsim manager |
| add_map_path | String | Specify the path to the yaml file of the additional map |
| train_mode | true, **false** | Since training mode is not yet implemented `false` should stay selected. (This would deactivate e.g. the task_generator) |
| disable_scenario | true, **false** | This parameter would e.g. disable the task generator and other selected notes. This should only be changed with caution |
| reset | **true**, false | Determines whether the scenario should be reset (or terminated) after reaching its objective (Robot has reached navigation goal) |
| enable_pedsim | **true**, false | Pedsim is used to for dynamic obstacle management. Setting this to false should disable dynamic obstacles |
| enable_pedvis | **true**, false | Responsible for converting pedsim messages, such as agent states, obstacles and providing their visualization in Rviz |


  # List of all Parameters
  ```xml
  <arg name="train_mode"        default="false"/>
  <arg name="local_planner"     default="dwa"         doc = "local_planer type [tep, dwa, rlca]"/>
  <arg name="rviz_file"         default="nav_LP"/>
  <arg name="disable_scenario"  default="false"/>
  <arg name="reset"             default="true"/>
  <arg name="enable_pedsim"     default="true"/>
  <arg name="outside"           default="false"/>
  <arg name="world"             default="aws_house"   unless="$(arg outside)"/>
  <arg name="world"             default="outside"     if="$(arg outside)"/>
  <arg name="model"             default="turtlebot3_burger" 
  <arg name="scenario_file"     default="$(arg world).json"/>
  <arg name="step_size"         default="0.01"/>
  <arg name="update_rate"       default="250"/>
  <arg name="show_viz"          default="true"/>
  <arg name="viz_pub_rate"      default="30" />
  <arg name="use_rviz"          default="true"/>
  <arg name="map_path"          default="$(find simulator_setup)/maps/$(arg world)/map.yaml" />
  <arg name="task_mode"         default="random"/> 
  <arg name="obs_vel"           default="0.3" />
  <arg name="action_frequency"  default="10"/>
  <arg name="global_frame_id"   value="map"/>
  <arg name="odom_frame_id"     value="odom"/>
  <arg name="base_frame_id"     value="base_footprint"/>
  <arg name="odom_ground_truth" default="/odometry/ground_truth"/>
  ```
