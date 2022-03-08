# Introduction

Arena-rosnav-3D is structurally very similar to arena-rosnav to allow easy switching between 2D and 3D environments.
Currently, three custom worlds and a random world generator are supported.
In our detailed custom worlds you can generate dynamic obstacles either in a scenario mode or in a random mode where the obstacle trajectories are determined randomly.

---

# Worlds

We provide the following worlds for your use:
|<img width="150" height="140" src="/img/aws_house-2.png">|<img width="150" height="140" src="/img/turtlebot3_house.png">|<img width="150" height="140" src="/img/small_warehouse-2.png">|<img width="150" height="140" src="/img/outside.png">|<img width="150" height="140" src="/img/factory.png">|<img width="150" height="140" src="/img/hospital.jpg">|
|:--: | :--:| :--:| :--:| :--:| :--:|
| _aws_house_ | _turtlebot3_house_ | _small_warehouse_ | _random world_ | _factory_ | _hospital_ |

| <img width="150" height="140" src="/img/experiment_rooms.jpg"> | <img width="150" height="140" src="/img/bookstore.jpeg"> | <img width="150" height="140" src="/img/tb_world.jpg"> |
| :------------------------------------------------------------: | :------------------------------------------------------: | :----------------------------------------------------: |
|                       _experiment_rooms_                       |                       _bookstore_                        |                   _turtlebot3_world_                   |

Furthermore, an outside world is available using the flag "outside:=true", e.g.

```
roslaunch arena_bringup start_arena_gazebo.launch outside:=true
```
## Calculated world properties
 | adjusted_mean | mean | variance | Entropy | Max-Entropy | MapSize | MinObsDis | NumObs | OccupancyRatio | obs_ratio | 
--- |--- | --- | --- |--- |--- |--- |--- |--- |--- |--- |
eval_floot | 0.054 | 317.647 | 5918.087 | 0.366 | 2.322 | 4000 | 0.224 | 18 | 0.0498 | 0.45 |
factory | - | 360 | 0 | 0.104 | 2.322 | 36000 | 0.223 | 70 | 0.009 | 0.194 |
hospital | 1.328 | 358.998 | 270.379 | 0.175 | 2.322 | 64000 | 0.224 | 82 | 0.019 | 0.128  |
ignc | 0.008 | 180 | 21406.596 | 0.146 | 2.322 | 4840 | 1000 | 1 | 0.017 | 0.020 |
small_warehouse | 0.008 | 200.930 | 26624.237 | 0.265 | 2.322 | 6250 | 0.224 | 30 | 0.032 | 0.48  |
aws_house | 0.182 | 351.628 | 1926.969 | 0.308 | 2.322 | 4000 | 0.224 | 41 | 0.037 | 1.025  |
bookstore | 0.006 | 114.545 | 18997.649 | 0.193 | 2.322 | 9000 | 0.224 | 59 | 0.019 | 0.655 |
exp_room | 0.033 | 270 | 8106.014 | 0.095 | 2.322 | 9000 | 1000 | 1 | 0.009 | 0.011 |
outside | - | 360 | 0 | 0.112 | 2.322 | 10240 | 1000 | 1 | 0.012 | 0.009 |
tb3_house | - | 360 | 0 |0.089 | 2.322 | 12250 | 0.4 | 5 | 0.008 | 0.040 |
tb3_world | 0.006 | 180 | 29083.099 |0.068 | 2.322 | 7372.8 | 0.75 | 10 | 0.006 | 0.135 |

## Pre-build world

To select one of our pre-build worlds, specify your scenario by selecting **aws_house**, turtlebot3*house, small_warehouse *(bold is your default world)\_ in your startup file:

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

# Robots

We support different robots:
|<img width="250" src="/img/robots/turtlebot3-burger.jpg">|<img width="250" src="/img/robots/jackal.jpg">|<img width="250"  src="/img/robots/ridgeback.jpg">|<img width="250" src="/img/robots/agv-ota.png">|
|:--: | :--:| :--:| :--:|
| _turtlebot3_burger_ | _jackal_ | _ridgeback_ | _agv-ota_ |

| <img width="250" src="/img/robots/rto.jpg"> | <img width="250" src="/img/robots/tiago.jpeg"> | <img width="250"  src="/img/robots/turtlebot3_waffle_pi.jpg"> | <img width="250" src="/img/robots/cob4.jpg"> |
| :-----------------------------------------: | :--------------------------------------------: | :-----------------------------------------------------------: | :------------------------------------------: |
|               _Robotino(rto)_               |                    _tiago_                     |                    _turtlebot3_waffle_pi_                     |             _Car-O-Bot4 (cob4)_              |

All robots are equipped with a laser scanner. The robots differ in size, laser-range etc. See below table for more detailed information on each robot:

| Name                   | Max Speed (v*x) [\_m/s*] | Max Speed (v*y) [\_m/s*] | Max Rotational Speed (θ*y) [\_rad/s*] | Radius [_m_] | Emergency-Stop¹ | Laser-range [_m_] | Holonomic² |
| :--------------------- | :----------------------: | :----------------------: | :-----------------------------------: | :----------: | :-------------: | :---------------: | :--------: |
| _turtlebot3-burger_    |           0.22           |           0.0            |                 2.84                  |    0.113     |      True       |        3.5        |   False    |
| _jackal_               |           2.0            |           0.0            |                  4.0                  |    0.267     |      True       |       30.0        |   False    |
| _ridgeback_            |           1.1            |           0.5            |                  2.0                  |    0.625     |      True       |       10.0        |    True    |
| _agv-ota_              |           0.5            |           0.0            |                  0.4                  |    0.629     |      True       |        5.0        |   False    |
| _rto³_                 |           2.78           |           todo           |                 todo                  |    0.225     |      todo       |       todo        |    todo    |
| _tiago_                |           1.5            |           todo           |                 todo                  |     0.27     |      todo       |       todo        |    todo    |
| _turtlebot3_waffle_pi_ |           0.26           |           0.0            |                 1.82                  |    0.208     |      False      |        3.5        |   False    |
| _Car-O-Bot4 (cob4)³_   |           1.1            |           todo           |                 todo                  |     0.36     |      True       |       todo        |    todo    |

For additional / more detailed information about each robot:

- [See the parameters needed for the **Navigation stack**](https://github.com/eliastreis/arena-rosnav-3D/tree/main/arena_navigation/arena_local_planer/model_based/conventional/config)
- [See additional robot parameters like laser min/max [_rad_]](https://github.com/eliastreis/arena-rosnav-3D/tree/main/arena_bringup/launch/sublaunch_testing/robot_params)
- See [_HERE_](https://github.com/eliastreis/arena-rosnav-3D/tree/main/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/configs) for the definition of the robots action_spaces (needed for rl-based-training)

> **_NOTE_**: The _emergency-stop_ capability is currently still being development, however, it will be available on all robots.

To select a robot model for your simulation run (in this case _ridgeback_):

```bash
roslaunch arena_bringup start_arena_gazebo.launch model:=ridgeback
```

¹ _Stops moving when an object has been detected in the safety zone of the robot_

² _For *non-holonomic* robots `vel_y = 0`; they are not able to drive directly to their left or right, but have to drive on a circular trajectory to their reach navigation goal_

³ _We only use the navigation platform of these robots_

---

# Navigation

Arena-rosnav-3d supports different local planner and task-modes. Select your planner to move to the respective documentation.

- [teb](#local-planner-teb-dwa-mpc-rlca-arena-aio)
- [dwa](#local-planner-teb-dwa-mpc-rlca-arena-aio)
- [mpc](#local-planner-teb-dwa-mpc-rlca-arena-aio)
- [rlca](#local-planner-teb-dwa-mpc-rlca-arena-aio)
- [aio](#local-planner-teb-dwa-mpc-rlca-arena-aio)
- [cadrl](#local-planner-cadrl)
- [drl](#local-planner-drl)
- [navrep](#local-planner-rosnav-navrep-guldenring)
- [gring](#local-planner-rosnav-navrep-guldenring)

Note, that not every robot can be used with every planner, yet. See an overview on possible cominations in the table below:

| Model  | TEB | DWA  | MPC  | ROSNAV | CADRL | RLCA | Gring | Navrep | ARENA | All-in-One |
| :--- | :---:|  :---: |:---: |:---: |:---:|   :---:| :---:| :---:| :---:|  :---: |
| *burger*   	 | X | X | X | X | X | X | X | X | X | X |
| *jackal*     | X | X | X | X | X | X | X | X |   | X |
| *ridgeback*  | X | X | X | X | X | X |   |   |   |   |
| *agv-ota*    | X | X | X | X | X |   |   |   |   |   |
| *rto*        | X | X | X | X | X |   | X | X |   | X |
| *youbot*     | X | X | X | X | X |   | X | X |   | X |
| *waffle_pi*  | X | X | X |   | X | X |   |   |   |   |
| *cob4*       | X | X | X | X | X |   |   | X |   |   |



Take note of the different task modes, which define how the start and goal position of the robot, as well as the trajectories of the obstacles will be set. Here we support the task modes:

- **random**: Here the start and goal position of the robot and the trajectories of the dynamic obstacles is determined randomly by arena-rosnavs _task-generator_ node.

  ```bash
  roslaunch arena_bringup start_arena_gazebo.launch task_mode:=random
  ```

- **scenario**: Here start and goal position of the robot, as well as the trajectory of dynamic obstacles are predefined in scenario files, which need to be selected at start up. You can define you own scenarios using the [arena-tools](https://github.com/ignc-research/arena-tools) repository. Here to start the scenario mode:
  ```bash
  roslaunch arena_bringup start_arena_gazebo.launch task_mode:=scenario scenario_file:=small_warehouse_obs05.json
  ```
- **manual**: Here you can set the start and goal position of the robot manually, using rviz `Goal Pose` button. Dynamic obstacles are not spawned.
  ```bash
  roslaunch arena_bringup start_arena_gazebo.launch task_mode:=manual
  ```

## Local-planner: teb, dwa, mpc, rlca, arena, aio

You can start these planners by setting the `local_planner` parameter in the launch file. As example, here how to start _teb_:

```bash
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=teb
```

> **NOTE**: The planer _rlca_ is currently only available for the robots: _turtlebot3_burger_, _turtlebot3_waffle_pi_

## Local-planner: cadrl

When first using the `cadrl`, refer to the documentation [here](/docs/Installation.md#cadrl) since you will need to create a different virtual environment.

To start the _cadrl_ planner run:

```bash
workon cadrl
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=cadrl
```

## Local-planner: rosnav, navrep, guldenring

When first using `rosnav` make sure to run the following commands:
```bash
pip install gym
cd ~/catkin_ws/src/forks/stable-baselines3 && pip install -e .
```

When first using `navrep` and `guldenring` make sure to follow the install documentation [here](/docker/drl_agent_node) first.

<!-- The rosnav planner hereby is our DRL agent. To extend the training and deployment package(s) please refer to the [DRL_Pipeline.md](/docs/Miscellaneous.md#How-to-include-further-scenarios). -->

# Parameters

| Name          | Type                                                  | Default | Description                                                                                                                                                                                                                                                                                                                                            |
| ------------- | ----------------------------------------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| task_mode     | random \| scenario                                    | random  | The task mode denotes if the robot start and goal position, as well as the trajectory of the dynamic obstacles is set randomly<br>In scenario mode all values are set beforehand. We currently support one scenario per world. To extend this and build your own custom scenarios see [here](/docs/Miscellaneous.md#How-to-include-further-scenarios). |
| local_planner | teb \| dwa \| mpc \| rlca \| cadrl \| arena \| rosnav | rosnav  | Denotes which planner is used to set the roboters _cmd_vel_                                                                                                                                                                                                                                                                                            |

## Additional parameters for DRL

| Name                  | Type                                                | Default           | Description                                                                                                                                                                         |
| --------------------- | --------------------------------------------------- | ----------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| network_type          | rosnav \| navrep \| guldenring                      | rosnav            | This is mainly needed to pick the right encoder. You can find a detailed documentation of how to add a new encoder and what this parameter exactly means [here](#TODO)              |
| trainings_environment | rosnav \| navrep \| guldenring                      | rosnav            | Indicates in which trainings environment the model you want to test has been trained. For all inputs except _rosnav_ the drl node is not started automatically.                     |
| agent_name            | string                                              | tb3               | Must exactly match the name of your model file. The model file needs to be in a directory named like the trainings_environment. More on this [here](https://github.com/ignc-research/arena-rosnav-3D/tree/main/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/agents)                          |
| model                 | turtlebot3_burger \| jackal \| ridgeback \| agv-ota | turtlebot3_burger | All roboter models we provide. Each roboter model has different velocities and scan sizes. Therefore, it is crucial that the model parameter matches the roboter your model is for. |

## Example

If you want to have a rosnav model called _rosnav_model_ control the _turtlebot3_burger_ robot your program call should look like this:

```bash
roslaunch arena_bringup start_arena_gazebo.launch model:=turtlebot3_burger local_planner:=rosnav world:=map5 scenario_file:=map5_obs_05.json agent_name:=turtlebot3_burger trainings_environment:=rosnav network_type:=rosnav
```

Whereas the arguments _trainings_environment_ and _network_type_ can be omitted for this use case, cause they are set by default.


<!-- ## Models from different training environments
Since _Navrep_ and _Guldenring_ are using a different Python version it is required to start the DRL Node seperately when using models trained in one of these environments.

Therefore, you can either start the node inside a virtual environment or use our provided Docker Containers. You can find a detailed documentation on how to build and run the Dockers [here](/docker/drl_agent_node) -->

---

# REST TODO

# Advanced Parameters

You can further customize the simulation altering the following parameters:

|       Name       |     Options     |                                                                                               Description                                                                                               |
| :--------------: | :-------------: | :-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|      actors      |     Integer     |                                  Specify the amount of actors to be inserted into a Gazebo world. All of the actors position are then managed by the pedsim simulator                                   |
|   actor_height   |     Double      | A fixed actor height on a per world basis needs to be set, default value of 1.1 is already set and should be appropriate for most of the worlds, if not experimentation for proper value will be needed |
|  additional_map  | true, **false** |                                                 Set to true, if you wish to use an additional, more detailed occupancy map for obstacles/pedsim manager                                                 |
|   add_map_path   |     String      |                                                                         Specify the path to the yaml file of the additional map                                                                         |
|    train_mode    | true, **false** |                                        Since training mode is not yet implemented `false` should stay selected. (This would deactivate e.g. the task_generator)                                         |
| disable_scenario | true, **false** |                                         This parameter would e.g. disable the task generator and other selected notes. This should only be changed with caution                                         |
|      reset       | **true**, false |                                    Determines whether the scenario should be reset (or terminated) after reaching its objective (Robot has reached navigation goal)                                     |
|  enable_pedsim   | **true**, false |                                                Pedsim is used to for dynamic obstacle management. Setting this to false should disable dynamic obstacles                                                |
|  enable_pedvis   | **true**, false |                                          Responsible for converting pedsim messages, such as agent states, obstacles and providing their visualization in Rviz                                          |

# List of all Parameters

```xml
<arg name="train_mode"        default="false"/>
<arg name="local_planner"     default="dwa"         doc = "local_planer type [tep, dwa, mpc, rlca, cadrl, arena, rosnav]"/>
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
