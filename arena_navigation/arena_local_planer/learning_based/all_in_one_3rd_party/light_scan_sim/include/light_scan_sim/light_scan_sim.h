/**
 * light_scan_sim light_scan_sim.h
 * @brief Monitor map and tf data, publish simulated laser scan
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 * 
 */

#ifndef LIGHT_SCAN_SIM_LIGHT_SCAN_SIM_H
#define LIGHT_SCAN_SIM_LIGHT_SCAN_SIM_H

#include <stdint.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include "light_scan_sim/ray_cast.h"

class LightScanSim {
  // Internal data
  nav_msgs::OccupancyGrid map_;
  light_scan_sim::MaterialList materials_;
  light_scan_sim::SegmentList segments_;
  tf::Transform map_to_image_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  bool map_loaded_ = false;
  bool segments_loaded_ = false;
  bool materials_loaded_ = false;

  // Publishers and subscribers
  ros::Subscriber map_sub_;
  ros::Subscriber materials_sub_;
  ros::Subscriber segments_sub_;
  ros::Publisher laser_pub_;
  std::shared_ptr<RayCast> ray_cast_;

  // Configurable options
  std::string map_topic_ = "/map";
  std::string materials_topic_ = "/map_materials";
  std::string segments_topic_ = "/map_segments";
  std::string laser_topic_ = "/scan";

  std::string image_frame_ = "/map_image";
  std::string laser_frame_ = "/initialpose";

  public:
    LightScanSim(ros::NodeHandle node);

    void MapCallback(const nav_msgs::OccupancyGrid::Ptr& grid);
    void MaterialsCallback(const light_scan_sim::MaterialList::Ptr& materials);
    void SegmentsCallback(const light_scan_sim::SegmentList::Ptr& segments);

    void Update();

};

#endif
