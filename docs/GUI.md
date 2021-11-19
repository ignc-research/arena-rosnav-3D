## Introduction
On top of the supplied roslaunch files, we also provide a Python GUI, put together as a more user-friendly approach to specifying the simulation parameters such as world map, task mode ...

![Picture of GUI](https://i.ibb.co/3C4XZjx/Screenshot-20211020-184556.png)
## Installation
In order to use our GUI please follow the installation steps in Installation.md. On top of that make sure you have these Python3 packages available on your system.

    $ sudo pip3 install pyqt5 interminal
    
## Run

  - Open terminal in arena-rosnav-3D directory and execute
  ``` 
python3 arena-rosnav-3D.py
  ```  

## Usage

This utility will look at all of the subfolders inside of /simulator_setup/worlds folder and list them as a simulation world choice.
Based on the selected world, a default value for some of the fields will be set, if corresponding files can be found inside of /simulator_setup folder. 

#### Parameters
- Additional map: here you can specify the path to a map yaml file, which will be used by our obstacle_manager node to manage the pedsim spawn/waypoints.
We give you the opportunity to supply a map, which will include obstacles at the height level of a Gazebo actor. Based on your use case, this can result in a more realistic simulation.
Please refer our guide on how to create a "layered" occupancy map [here](https://github.com/Jacenty00/gazebo_ros_2Dmap_plugin)

- Actor height: As the pedsim agents are simulated in a 2D environment, we have to specify a fixed actor height to be used by our Gazebo plugin. As such you may have to change the default value of 1.1, depending on your Gazebo world.

- Pedsim scene: xml file, which determines the position of obstacles to be considered by pedsim during the simulation. On how to convert Gazebo worlds to scene filse, please refer to (link).

For a further explanation on all of the available parameters, please refer to [Usage.md](docs/Usage.md)
