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

# How to automate the scenario mode (for large scale benchmarking)
For running multiple scenario files, (on different robots and planers), we provide the `launch_arena.py` file. This will automatically _roslaunch_ the scenario files and save the respective _.csv_ files. Use this as follows:

1. create a _config.yaml_ in which you specify: _planer, robot, scenario, etc._. You can find an example config file [here](https://github.com/ignc-research/arena-rosnav-3D/blob/main/arena_bringup/launch/launch_configs/example_config.yaml). The script will later automatically launch every unique combination of planer, robot and scenario. You can also turn of the visualization (to increase simulation speed, by setting Visualization to `False`)
2. save the file under: `arena-rosnav-3D/arena_bringup/launch/{NAME-OF-YOUR-CONFIG}.yaml` and copy the path to the config
3. [OPTIONAL:] Your might want to change further parameters since for example you want to active a different _virtual environement_ for certain planners. You can do this [here](https://github.com/ignc-research/arena-rosnav-3D/blob/main/arena_bringup/launch/launch_arena.py)
4. Launch the file, by providing your PATH_TO_YOUR_CONFIG (from step 1) as argument:
```zsh
roscd arena_bringup && cd launch
python launch_arena.py --yaml_path {PATH_TO_YOUR_CONFIG}
```
> __NOTE__: If you manually terminate the script, ros and gazebo might still continue to run. Terminate them by entering the following commands in the terminal:
```
killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient
rosnode kill --all
```
# How to include additional robot models
> __NOTE__: Since every robot model works differently it is not possible to provide here a comprehensive guide. We provide only some important steps, and tests to ensure your robot model is implemented properly.

__General Tips:__

❗Make sure your: `YOUR_ROBOT_NAME` stays consistent with every use\
❗Use an underscore for multi-word robot names, like: `robot_name`

__Implementation:__
1. Gazebo uses the _.urdf_ format to define gazebo-robot model. Add the corresponding fils to: `simulator_setup/robot`.\
__Note:__ The _.urdf_ model will sometimes read out files from other (support) packages / files. Make sure to update these paths accordingly by looking at all appearances of `package://` and `$(find ` in your newly added files.
2. Since some robot models require extra packages for example to map the laser scan data. You should make sure to include them im the `robot.launch` file ([here](https://github.com/ignc-research/arena-rosnav-3D/blob/87258d562292db7a006326eac8069998fea717c9/arena_bringup/launch/sublaunch_testing/robots.launch#L2)). You can use the *group_by* parameter to only activate the node in the case of your robot model.
__Note:__ To check weather your robot is implemented correctly, make sure a laser scan topic is published under the name `scan`. Run: 
    ```bash
    rostopic echo scan
    ```
    If this is the case, check also weather *tf* is setup correctly, (by opening **rviz** > **Add** > **LaserScan** and writing into 'topic': `/scan`. (If your scan topic is published under some other name, change this to *scan* since, this is required by other arena-rosnav modules)

3. If you want to use _classical_ planers (like _teb_, _dwa_ or _mpc_), you need to add their respective parameters, under:\
`arena_navigation/arena_local_planer/model_based/conventional/config/{YOUR_ROBOT_NAME}`\
You can also check the launch files of the respective planers like for example [here](https://github.com/ignc-research/arena-rosnav-3D/blob/main/arena_bringup/launch/sublaunch_testing/move_base/move_base_dwa.launch) to see the needed files. 
4. Make sure to add also a parameter file (to be published to the parameter sever), under:\
`arena_bringup/launch/sublaunch_testing/robot_params/{YOUR_ROBOT_NAME}_params.yaml`
    - The parameters: [*robot_action_rate, laser_update_rate, laser_min, laser_max, laser_range, laser_beams*] can (usually) be found in the _.urdf_ file(s) of the robot model. 
    - If the *radius* is not given you can approximate the max. radius for example by calculating it by the data given in the 'footprint' section of the `costmap_common_params.yaml` file.
    - The *speed* parameter can often be found on the website of the manufacturer, or in some additional config files.
    - The laser increment can be calculated by *(laser_max-laser_min)/laser_beams*
5. If you want to use this robot for drl training you must also add the definition of the actionspace of the robot under:\
`arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/configs`
6. If you want to make your implementation publicly available, make sure to update the documentation [here](Usage.md#robots) and [here](https://github.com/ignc-research/arena-rosnav-3D#robots)
