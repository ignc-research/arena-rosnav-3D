/**
 * light_scan_sim ray_cast.cpp
 * @brief Cast simulated laser rays on an image.
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#include "light_scan_sim/ray_cast.h"
#include <algorithm>

/**
 * @brief Return the first collision point (hit) and true, or false if no collision
 *
 * @param start The start point. Will be clipped to image.
 * @param end   The end point. Will be clipped to image.
 * @param hit   The collision point
 *
 * @return true if there was a collision or false
 */
bool RayCast::Trace(cv::Point2f &start, cv::Point2f &end, cv::Point2f &hit) {

  cv::Point2i start_d = start;  // Opencv casting rounds rather than truncating
  cv::Point2i end_d = end;

  // Ensure that the line is in the map
  if (!cv::clipLine(map_.size(), start_d, end_d)) {
    return false;
  }

  // Iterate from start to end
  cv::LineIterator it(map_, start_d, end_d, 4);  // 8 way connectivity, smoother than 4 way
  for(int i = 0; i < it.count; i++, ++it) {
    if (map_.at<uint8_t>(it.pos())>0) {
      hit = it.pos();
      return true;
    }
  }
  return false;
}

/**
 * @brief Create a simulated laser scan
 * 
 * @param start The origin point. px
 * @param yaw The origin angle of the scan
 *
 * @return a laser scan message
 */
sensor_msgs::LaserScan RayCast::Scan(cv::Point2f start, double yaw) {
  sensor_msgs::LaserScan scan;

  scan.angle_min = angle_min_;
  scan.angle_max = angle_max_;
  scan.range_min = ray_min_;
  scan.range_max = ray_max_;
  scan.angle_increment = angle_inc_;

  cv::Point2f hit;
  double max_px = ray_max_/m_per_px_;

  for (double a = angle_min_; a <= angle_max_; a+=angle_inc_) {
    cv::Point2f end = cv::Point2f(start.x + max_px*cos(yaw+a),
                                  start.y + max_px*sin(yaw+a));
    
    if (Trace(start, end, hit)) {
      double range = cv::norm(hit-start);  // distance from start to hit
      range *= m_per_px_;  // convert back to m

      // Check for collision with wall segments
      double start_x_m = start.x*m_per_px_ + map_offset_.x;
      double start_y_m = start.y*m_per_px_ + map_offset_.y;
      if (wall_segments_) {
        wall_segments_->Trace(start_x_m, start_y_m, yaw+a, range, ray_max_, range);
      }

      // ROS_INFO_STREAM("Outside: " << range);

      if (range < ray_min_) {
        range = ray_max_ + 1.0;
      }

      scan.ranges.push_back(range);
    } else {
      scan.ranges.push_back(ray_max_+1.0);  // Out of range, represented by value>max
    }
  }

  return scan;
}
