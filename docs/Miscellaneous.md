# How to include further world files

1. Download the world-file + models (if needed)
2. The `.world` file should be saved in [simulator_setup/worlds](simulator_setup/worlds). Create a file of the following structure:

```
├── simulator_setup/
|    └── wolds/
│        └── {NAME OF YOUR WORLD}
│            └── models/ # if needed
|            └── worlds/
|                └── {NAME OF YOUR WORLD}.world
```    

3. If your world includes further gazebo models set the `GAZEBO_MODEL_PATH` by appending the following line in the arena_simulator `package.xml` file (in between the `<export></export>` tags)
```xml
 <gazebo_ros plugin_path="${prefix}/worlds/{NAME OF YOUR WORLD}/lib" gazebo_media_path="${prefix}worlds/{NAME OF YOUR WORLD}" gazebo_model_path="${prefix}/worlds/{NAME OF YOUR WORLD}/models"/>
```
4. Create a  occupancy map-file, safe it under `simulator_setup/maps/map_{NAME OF YOUR WORLD}/map.pgm`. If the map is not already created you can create it by using: 

- [Gmapping](http://wiki.ros.org/gmapping)
- [gazebo_ros_2d_map_plugin](https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin): automatically generates a 2D occupancy map from gazebo
- [birdview_map]() to create 2D map that includes obstacles above robot hight (which might effect dynamic obstacle)

5. Create a pedsim-scenario by using [this](https://github.com/fverdoja/ros_maps_to_pedsim) package and your (created) .map file. The pedsim-scenario should be saved under `simulator_setup/scenarios/ped_scenarios/{NAME_OF_YOUR_WORLD}.xml`
   
   Your launch file for this will probably look something like this:
   ```bash
   roslaunch ros_maps_to_pedsim ros_maps_to_pedsim.launch map_path:='{path to the folder}' use_map_origin:='true' add_agents:='false'
   ```

6. Build arena-rosnav-3D with `catkin_make`
7. Start arena-rosnav-3D with your world file included by running the following command in the terminal:
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


1. Create your own scenario using [arena-tools](https://github.com/Jacenty00/arena-tools#scenario-editor)
2. Save the file under the path `simulator_setup/scenarios/{NAME_OF_YOUR_SCENARIO}.json`. In case you create multiple scenarios per world you will have to specify your scenario at startup like this:
   
```bash
roslaunch arena_bringup start_arena_gazebo.launch scenario_file:=simulator_setup/scenarios/{NAME_OF_YOUR_SCENARIO}.json
```

<ins>NOTE:</ins> The number of indoor dynamic obstacles should be chosen wisely. In confined environments such as the turtlebot3_house, too many obstacles will result in uneven trajectories (like obstacles getting stuck etc).

# How to create more world files
1. Gazebo provides a well done documentation on how to build a base world, see [here](http://gazebosim.org/tutorials?tut=building_editor&cat=build_world). This allows you to effortlessly build walls, doors and windows (based on a floor plan) (on some computers the build editor crashes when you try to add windows and doors, in which case just skip this step (leave space in the wall for possible doors)).

    <ins>Tip:</ins> 
    
    When working with the gazebo editor, make sure to save your progress regularly since the gazebo editor does have a tendency to randomly shutdown at times.

2. To populate the world further, for example with furniture, see [this](http://gazebosim.org/tutorials?tut=model_editor&cat=build_robot) tutorial. If you're new to Gazebo, you might also want to check out [this](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b2) tutorial on the user interface. <br>
You can also download further models from [here](https://app.ignitionrobotics.org/dashboard). Via the download function. Extract the downloaded models to `~/.gazebo/models` so you can use them in gazebo.

    <ins>Tip:</ins>

    a) Not all Gazebo models that are available actually work. Due to the different Gazebo versions, opening some models may cause your program to crash. It is therefore advisable to test your model in an empty Gazebo world to see if it works properly.

    b) If you want to use our model multiple times. You can just copy and past it like a normal string.

3. If you intend to include pedestrians in your model, you can do so using the Actor concept, as described [here](http://gazebosim.org/tutorials?tut=actor&cat=build_robot). To include a walking pedestrian insert the following lines in between your `<world></world>` tags:
   ```xml
       <actor name="actor1">
      <skin>
        <filename>walk.dae</filename>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <trajectory id="0" type="walking">
          <waypoint>
            <time>0</time>
            <pose>-4 -9 0 0 0 -0.49</pose>
          </waypoint>
          <waypoint>
            <time>10</time>
            <pose>2 -10 0 0 0 -0.49</pose>
          </waypoint>
          <waypoint>
            <time>12</time>
            <pose>2 -10 0 0 0 2.5</pose>
          </waypoint>
          <waypoint>
            <time>20</time>
            <pose>-4 -9 0 0 0 2.5</pose>
          </waypoint>
          <waypoint>
            <time>22</time>
            <pose>-4 -9 0 0 0 -0.49</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>
   ```
    For multiple actors change `name="actor1"` and ` id="0"` as well as the trajectory. 
    The trajectory is defined by its waypoints, which are in the following format: `<pose>x y z roll pitch yaw</pose>`. If you want to give the actor a different trajectory, you can change the waypoints. 

    You can test the changes to your world by running in the terminal:
    ```bash
    gazebo {path to your world}/{name of your world}.world
    ```
        
    <ins>Tip:</ins> 
    
    a) The actor can only walk straight, so if you wan to change its position, you have to change yaw angle as well. See [here](https://answers.ros.org/question/141366/convert-the-yaw-euler-angle-into-into-the-range-0-360/) for conversion
    
    b) Take note of your current x and y axis. Depending on your settings these might be changed


To include your custom world in arena-rosnav-3D, see [here](#How-to-include-further-world-files).
If you are using models, downloaded from ignitionrobotics you need to copy their folders from `~/.gazebo/models` to `~/catkin_ws/src/arena-rosnav-3D/simulator_setup/worlds/small_warehouse/models` so they are included with the simulator.

# How to speed-up gazebo simulation speed
It is possible to run Gazebo faster than real time. The maximum simulation speed depends on the complexity of the world and the processing power, a speedup between 2 and 300 of the simulation time is possible.
- To speed up the simulation time to the maximum capacity, go into the world file of the gazebo (for the small warehouse world this would be under `simulator_setup/worlds/small_warehouse/worlds`) and change the `real_time_update_rate` from `<real_time_update_rate>1000</real_time_update_rate>` to `<real_time_update_rate>0</real_time_update_rate>`. This will speed up your simulation and adjust all parameters. (If you want to increase the simulation by a certain factor, see [here](http://gazebosim.org/tutorials?tut=physics_params&cat=physics)). You can further increase the speed by running the simulation headless. To do this, follow the description [here](https://github.com/eliastreis/arena-rosnav-3D/blob/2ecc4640576fce3e39356f5dc806b7d1986ed493/arena_bringup/launch/sublaunch_testing/gazebo_simulator.launch#L17)).
