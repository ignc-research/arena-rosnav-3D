# light_scan_sim

[![Build Status](http://build.ros.org/job/Idev__light_scan_sim__ubuntu_trusty_amd64/badge/icon)](http://build.ros.org/job/Idev__light_scan_sim__ubuntu_trusty_amd64)

A very lightweight 2d laser scan simulator for ROS.

This is designed as an alternative to running Gazebo, or Player Stage. 

Here's a basic rundown:
- Input: a tf transform representing your laser's position (base_laser_link or laser_joint on a TurtleBot for example)
- Input: An OccupancyGrid (you can publish this using a [map_server](http://wiki.ros.org/map_server) node)
- Output: A [LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) against the OccupancyGrid

## Quickstart:
- git clone this into your catkin workspace
- catkin_make
- roslaunch light_scan_sim test.launch
- use the 2D Pose Estimate tool in RViz to set the simulated laser position

## Features:
- Gaussian noise (optional, disable by setting laser/noise to 0)
- Full RosParam Configuration
- Unit tests (incomplete)
- Vector obstacles
- Glass

## Configurable Parameters:
- range/min: Min laser range (double, default: 1.0). Values closer will be set to range/min value.
- range/max: Max laser range (double, default: 1.0). Values farther will be set to range/max + 1.0 (no return)
- range/noise: Gaussian noise standard deviation (double, default: 0.01). Set to 0 to disable noise.
- angle/min: Scan start angle (double, default -π/2)
- angle/min: Scan end angle (double, default π/2, inclusive)
- angle/increment: Angular step size (double, default 0.01)
- **map/topic**: The OccupancyGrid topic to listen to (string, default "/map")
- map/image_frame: An intermediate TF frame published internally (string, default "/map_image")
- laser/hz: laser scan frequency (double, default: 40.0)
- **laser/topic**: The LaserScan topic to publish to (string, default "/scan")
- **laser/frame**: The tf frame that the LaserScan is attached to  (string, default "/initialpose")

## License

MIT License

Copyright (c) 2017 Joseph Duchesne

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
