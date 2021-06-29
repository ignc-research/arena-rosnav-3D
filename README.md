# arena-rosnav-3D

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
