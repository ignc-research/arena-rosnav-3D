/**
 * light_scan_sim light_scan_sim_node.cpp
 * @brief A ROS node that publishes laser scans
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#include <ros/ros.h>
#include "light_scan_sim/light_scan_sim.h"

/**
 * @brief Init node and update the sim at the desired rate
 */
int main(int argc, char** argv){
  ros::init(argc, argv, "light_scan_sim");

  ros::NodeHandle node("~");

  LightScanSim sim(node); 

  ros::Rate rate(node.param<double>("laser/hz", 40.0));

  while (node.ok()){
    ros::spinOnce();  // Read any waiting messages

    sim.Update();  // Update the simulated laser scan

    rate.sleep();  // Wait until the 
  }

  return 0;
};
