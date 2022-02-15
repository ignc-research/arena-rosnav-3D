/**
 * light_scan_sim light_scan_sim.cpp
 * @brief Monitor map and tf data, publish simulated laser scan
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 * 
 */

#include "light_scan_sim/light_scan_sim.h"
#include <math.h>

/**
 * @brief Initialize light scan sim class
 *
 * @param node The ros node handle
 */
LightScanSim::LightScanSim(ros::NodeHandle node) {

  // Load settings
  ray_cast_ = std::make_shared<RayCast>(node.param<double>("range/min", 1.0),
                                        node.param<double>("range/max", 20.0),
                                        node.param<double>("angle/min", -M_PI_2),
                                        node.param<double>("angle/max", M_PI_2),
                                        node.param<double>("angle/increment", 0.01),
                                        node.param<double>("range/noise", 0.01));

  node.getParam("map/topic", map_topic_);
  node.getParam("map/materials_topic", materials_topic_);
  node.getParam("map/segments_topic", segments_topic_);
  node.getParam("laser/topic", laser_topic_);
  node.getParam("map/image_frame", image_frame_);
  node.getParam("laser/frame", laser_frame_);

  // Subscribe / Publish
  map_sub_ = node.subscribe(map_topic_, 1, &LightScanSim::MapCallback, this);
  materials_sub_ = node.subscribe(materials_topic_, 1, &LightScanSim::MaterialsCallback, this);
  segments_sub_ = node.subscribe(segments_topic_, 1, &LightScanSim::SegmentsCallback, this);
  laser_pub_ = node.advertise<sensor_msgs::LaserScan>(laser_topic_, 1);
}

/**
 * @brief Recieve the subscribed map and process its data
 *
 * @param grid The map occupancy grid
 */ 
void LightScanSim::MapCallback(const nav_msgs::OccupancyGrid::Ptr& grid)
{
  map_ = *grid;  // Copy the entire message
  
  // Convert OccupancyGrid to cv::Mat, uint8_t
  cv::Mat map_mat = cv::Mat(map_.info.height, map_.info.width,
                            CV_8UC1, map_.data.data());
  // Set unknown space (255) to free space (0)
  // 4 = threshold to zero, inverted
  // See: http://docs.opencv.org/3.1.0/db/d8e/tutorial_threshold.html
  cv::threshold(map_mat, map_mat, 254, 255, 4); 

  // Update map
  ray_cast_->SetMap(map_mat, map_.info.resolution, map_.info.origin.position.x, map_.info.origin.position.y);
  
  // Create transform from map tf to image tf
  map_to_image_.setOrigin(tf::Vector3(map_.info.origin.position.x,
                                      map_.info.origin.position.y,
                                      map_.info.origin.position.z));
  // Image is in standard right hand orientation
  map_to_image_.setRotation(tf::createQuaternionFromRPY(0, 0, 0));

  map_loaded_ = true;
  if (map_loaded_ && segments_loaded_ && materials_loaded_) {
    ray_cast_->SetSegments(segments_, materials_);
  }
}

/**
 * @brief Load materials and set segments/materials on ray_cast_ if possible
 *
 * @param materials The material list
 */
void LightScanSim::MaterialsCallback(const light_scan_sim::MaterialList::Ptr& materials) {
  materials_ = *materials;
  materials_loaded_ = true;

  if (map_loaded_ && segments_loaded_ && materials_loaded_) {
    ray_cast_->SetSegments(segments_, materials_);
  }
}

/**
 * @brief Load segments and set segments/materials on ray_cast_ if possible
 *
 * @param segments The segment list
 */
void LightScanSim::SegmentsCallback(const light_scan_sim::SegmentList::Ptr& segments) {
  segments_ = *segments;
  segments_loaded_ = true;

  // Todo: Somehow use TF to transform segments into image space

  if (map_loaded_ && segments_loaded_ && materials_loaded_) {
    ray_cast_->SetSegments(segments_, materials_);
  }
}

/**
 * @brief Generate and publish the simulated laser scan
 */
void LightScanSim::Update() {
  if (!map_loaded_) {
    ROS_WARN("LightScanSim: Update called, no map yet");
    return;
  }

  // Broadcast the tf representing the map image
  tf_broadcaster_.sendTransform(
    tf::StampedTransform(map_to_image_, ros::Time::now(),
                         map_.header.frame_id, image_frame_));

  // Use that transform to generate a point in image space
  tf::StampedTransform image_to_laser;
  try{
    tf_listener_.lookupTransform(image_frame_, laser_frame_,
                                 ros::Time(0), image_to_laser);
  } catch (tf::TransformException &ex) {
    ROS_WARN("LightScanSim: %s",ex.what());
    return;
  }

  // Convert that point from m to px
  cv::Point laser_point(image_to_laser.getOrigin().x()/map_.info.resolution,
                        image_to_laser.getOrigin().y()/map_.info.resolution);
  // And get the yaw
  double roll, pitch, yaw;
  image_to_laser.getBasis().getRPY(roll, pitch, yaw);

  // Generate the ray cast laser scan at that point and orientation
  sensor_msgs::LaserScan scan = ray_cast_->Scan(laser_point, yaw);

  // Set the header values
  scan.header.stamp = image_to_laser.stamp_;  // Use correct time
  scan.header.frame_id = laser_frame_;  // set laser's tf

  // And publish the laser scan
  laser_pub_.publish(scan);
}
