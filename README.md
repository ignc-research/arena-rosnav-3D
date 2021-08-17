# arena-rosnav-3D
This repository combines the 3D ROS simulator Gazebo with Pedsim to provide realistic dyanmic 3D scenarios and tasks to evaluate and and benchmark ROS navigation approaches. It is fully compatible with the planning algorithms trained and developed with arena-rosnav (2D). This presents an essential step in deploying the navigation approaches from arena-rosnav towards real robots. 

The repo currently contains:
* a 3D task generator with 3 modes: random, scenario and manual tasks
* realistic 3D scenarios from AWS
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

### Helpful links
* https://github.com/aws-robotics/aws-robomaker-hospital-world
* https://github.com/aws-robotics/aws-robomaker-small-warehouse-world


Human Gazebo

* https://github.com/robotology/human-gazebo



## First Steps
* Do ROS1 tutorials (we are working with ubuntu 18 and ROS melodic! :
http://wiki.ros.org/ROS/Tutorials

* Install Gazebo Robot simulator and do some tutorials to understand gazebo: 
Start with this:
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
* Tutorials for advanced functionalities later on (spawn obstacles, build world, etc.):
http://gazebosim.org/tutorials

http://wiki.ros.org/simulator_gazebo/Tutorials/SpawningObjectInSimulation

* install the arena-rosnav environment:
https://github.com/ignc-research/arena-rosnav

* Understand the task generator (you can have a look, copy and adjust the code for gazebo, whereas arena-rosnav uses flatland as simulation engine to spawn obstacles, this should now be achieved in gazebo):
https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/task_generator/


## Follow-up Steps
* Build some complex environments of offices, warehouses, etc. (see existing world files and links above)
* Integrate humans and dynamic obstacles 
Therefore, have a look at these gazebo pluggins: 
    * https://github.com/onlytailei/gym_ped_sim
    * https://github.com/srl-freiburg/pedsim_ros

    Also have a look at the Gazebo actor documentation 
    http://gazebosim.org/tutorials?tut=actor&cat=build_robot
    
* Implement/integrate the tasks generator of arena-rosnav into gazebo to have different spawn modes:
    * random spawning of dynamic obstacles
    * scenario spawning (where we can set specific scenarios)
    * reset functionality, where the robot resets after reaching a goal (this is improtatnt to conduct consistant evaluation runs which consist of e.g. 50 runs where the robot has to reach the goal 50 times in order to evaluate its performance
* Test some arena-rosnav planners in Gazebo
