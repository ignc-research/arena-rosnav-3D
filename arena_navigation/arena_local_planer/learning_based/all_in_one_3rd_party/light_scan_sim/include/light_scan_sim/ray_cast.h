/**
 * light_scan_sim ray_cast.h
 * @brief Cast simulated laser rays on an image.
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#ifndef LIGHT_SCAN_SIM_RAY_CAST_H
#define LIGHT_SCAN_SIM_RAY_CAST_H

#include <sensor_msgs/LaserScan.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "light_scan_sim/wall_segment_sim.h"

#include <random>

class RayCast {
  std::default_random_engine random_generator_;
  cv::Mat map_;
  std::shared_ptr<WallSegmentSim> wall_segments_ = nullptr;
  double m_per_px_;
  cv::Point map_offset_;

  // Default configuration variables
  double ray_min_ = 1.0;  // m
  double ray_max_ = 20.0;  // m

  double angle_min_ = -M_PI_2;
  double angle_max_ = M_PI_2;
  double angle_inc_ = 0.01;  // rad

  double noise_std_dev_ = 0.0;  // std. deviation of laser noise

  public:
    RayCast() {}
    RayCast(double ray_min, double ray_max,
            double angle_min, double angle_max, double angle_inc, double noise) {
      ray_min_ = ray_min;
      ray_max_ = ray_max;
      angle_min_ = angle_min;
      angle_max_ = angle_max;
      angle_inc_ = angle_inc;
      noise_std_dev_ = noise;
    };

    void SetMap(cv::Mat& map, double m_per_px, double offset_x, double offset_y) {
      map_offset_ = cv::Point(offset_x, offset_y);
      map_ = map;
      m_per_px_ = m_per_px;
    };

    void SetSegments(light_scan_sim::SegmentList &segments, light_scan_sim::MaterialList &materials) {
      wall_segments_ = std::make_shared<WallSegmentSim>(segments, materials);
    }

    bool Trace(cv::Point2f &start, cv::Point2f &end, cv::Point2f &hit);
  
    sensor_msgs::LaserScan Scan(cv::Point2f start, double yaw);
};

#endif

