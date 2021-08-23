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
- [gazebo_ros_2Dmap_plugin](https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin): automatically generates a 2D occupancy map from gazebo
- [birdview_map]() to create 2D map that includes obstacles above robot hight (which might effect dynamic obstacle)

6. Create a pedsim-scenario by using this package and your (created) .map file. The pedsim-scenario should be saved under `simulator_setup/scenarios/ped_scenarios/{NAME_OF_YOUR_WORLD}.xml`

 
7. Build arena-rosnav-3D with `catkin_make`
8. Start arena-rosnav-3D with your world file included by running the following command in the terminal:
```bash
roslaunch arena_bringup start_arena_gazebo.launch world:={NAME_OF_YOUR_WORLD}
```

<ins>NOTE:</ins> The {NAME_OF_YOUR_WORLD} must be consistent!

# How to include further scenarios

NOTE: The number of indoor dynamic obstacles should be chosen wisely. In confined environments such as the turtlebot3_house, too many obstacles will result in uneven trajectories (like obstacles getting stuck etc).