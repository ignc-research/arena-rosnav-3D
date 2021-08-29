# How to include further world files

1. Download the world-file + models (if needed)
2. The `.world` file should be saved in [simulator_setup/worlds](simulator_setup/worlds). Create a file of the following structure:

```
├── simulator_setup/
|    └── wolds/
│        └── {NAME OF YOUR WORLD}
│            └── models/ # if needed
|            └── worlds/
|                └── {NAME OF YOUR WORLD}.word
```    
3. Insert the following line into your model between the `<world></world>` tags to enable dynamic obstacle steering via pedsim_ros

```xml
<plugin name="ActorPosesPlugin" filename="libActorPosesPlugin.so"/>
```
4. If your world includes further gazebo models set the `GAZEBO_MODEL_PATH` by appending the following line in the arena_simulator `package.xml` file (in between the `<export></export>` tags)
```xml
 <gazebo_ros plugin_path="${prefix}/worlds/{NAME OF YOUR WORLD}/lib" gazebo_media_path="${prefix}worlds/{NAME OF YOUR WORLD}" gazebo_model_path="${prefix}/worlds/{NAME OF YOUR WORLD}/models"/>
```
5. Create a  occupancy map-file, safe it under `simulator_setup/maps/map_{NAME OF YOUR WORLD}/map.png`. If the map is not already created you can create it by using: 

- [Gmapping](http://wiki.ros.org/gmapping)
- [gazebo_ros_2d_map_plugin](https://github.com/marinaKollmitz/gazebo_ros_2d_map_plugin): automatically generates a 2D occupancy map from gazebo
- [birdview_map]() to create 2D map that includes obstacles above robot hight (which might effect dynamic obstacle)

6. Create a pedsim-scenario by using this package and your (created) .map file. The pedsim-scenario should be saved under `simulator_setup/scenarios/ped_scenarios/{NAME_OF_YOUR_WORLD}.xml`

 
7. Build arena-rosnav-3D with `catkin_make`
8. Start arena-rosnav-3D with your world file included by running the following command in the terminal:
```bash
roslaunch arena_bringup start_arena_gazebo.launch world:={NAME_OF_YOUR_WORLD}
```

<ins>NOTE:</ins> The {NAME_OF_YOUR_WORLD} must be consistent!


# How to include further scenarios
We currently provide one scenario per world. To you can extent this by

- include more dynamic obstacles
- alter obstacle behavior like speed (vmax)
- alter robot goal behavior

To create a new scenario for a specific world

1. Find the current scenario under `simulator_setup/scenarios/{NAME_OF_YOUR_WORLD}.json`
2. Create your own scenario on basis of the current scenario. (You can also look into other scenario files to find other predefined dynamic obstacles). (Keep in mind the inverted oriented orientation of X and Y axis at Gazebo-startup)
3. Save the file under the above path. In case you create multiple scenarios per world you will have to specify your scenario at startup like this:
   
```bash
roslaunch arena_bringup start_arena_gazebo.launch scenario_file:=simulator_setup/scenarios/prefix_{NAME_OF_YOUR_WORLD}.json
```

<ins>NOTE:</ins> The number of indoor dynamic obstacles should be chosen wisely. In confined environments such as the turtlebot3_house, too many obstacles will result in uneven trajectories (like obstacles getting stuck etc).

# Things to know when integrating arena-rosnav (2D) planer
Arena-rosnav-3D does set the robot's goal position slightly different then arena-rosnav (see the code [here](https://github.com/eliastreis/arena-rosnav-3D/blob/9642467f01ba8704f65693d185d468e361cfb747/task_generator/task_generator/robot_manager.py#L62). Arena-rosnav uses as for goal publishing `PoseStamped` as message type for the navigation goal. Arena-rosnav-3D uses `MoveBaseGoal` (since it includes a third dimension (z)).

Import the following line:
```python
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
```
replacing `PoseStamped` by .. usually should do the trick.

^^ This is not true. The problem lies with the time space planer. (I think it takes the wrong message type)


see: arena_navigation/arena_timespace_planner

# How to create more world files
Gazebo provides a well done documentation on how to build a base world, see [here](http://gazebosim.org/tutorials?tut=building_editor&cat=build_world). This allows you to effortlessly build walls, doors and windows (based on a floor plan) (on some computers the build editor crashes when you try to add windows and doors, in which case just skip this step (leave space in the wall for possible doors)).

To populate the world further, for example with furniture, see [this](http://gazebosim.org/tutorials?tut=model_editor&cat=build_robot) tutorial. If you're new to Gazebo, you might also want to check out [this](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b2) tutorial on the user interface. 

If you intend to include pedestrians in your model, you can do so using the Actor concept, as described [here](http://gazebosim.org/tutorials?tut=actor&cat=build_robot).

To include your custom world in arena-rosnav-3D, see [here](docs/Miscellaneous.md#How-to-create-more-world-files).

# How to speed-up gazebo simulation speed
It is possible to run Gazebo faster than real time. The maximum simulation speed depends on the complexity of the world and the processing power, a speedup between 2 and 300 of the simulation time is possible.
- To speed up the simulation time to the maximum capacity, go into the world file of the gazebo (for the small warehouse world this would be [here](simulator_setup/worlds/small_warehouse/worlds)) and change the `real_time_update_rate` from `<real_time_update_rate>1000</real_time_update_rate>` to `<real_time_update_rate>0</real_time_update_rate>`. This will speed up your simulation and adjust all parameters. (If you want to increase the simulation by a certain factor, see [here](http://gazebosim.org/tutorials?tut=physics_params&cat=physics)). You can further increase the speed by running the simulation headless. To do this, follow the description [here](arena_bringup/launch/sublaunch_testing/gazebo_simulator.launch).