/**
 * light_scan_sim test_ray_cast.cpp
 * @brief Test simulating laser rays on an image.
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#include "light_scan_sim/ray_cast.h"
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/LaserScan.h>

// The fixture for testing class RayCast
class RayCastTest : public ::testing::Test {};

TEST_F(RayCastTest, TraceTest) {
  RayCast rc;
  cv::Point2f start, end, hit;

  // Set up the raycast with a very simple map
  cv::Mat mat = cv::Mat::zeros(20, 20, CV_8UC1); 
  cv::rectangle( mat, cv::Point( 10, 0 ), cv::Point( 20, 20), 255, CV_FILLED);
  rc.SetMap(mat, 1.0, 0, 0);

  // Test tracing from empty space into wall
  start = cv::Point2f(5,5);
  end = cv::Point2f(15,5);
  EXPECT_TRUE(rc.Trace(start, end, hit));
  EXPECT_NEAR(hit.x, 10, 1e-5);
  EXPECT_NEAR(hit.y, 5, 1e-5);
  
  // Test tracing out of map into empty space
  start = cv::Point2f(5,5);
  end = cv::Point2f(-5,5);
  EXPECT_FALSE(rc.Trace(start, end, hit));
}


TEST_F(RayCastTest, ScanTest) {
  RayCast rc(0, 50, -M_PI_2, M_PI_2, M_PI_2, 0);
  cv::Point2f start(5,5);

  // Set up the raycast with a very simple map
  cv::Mat mat = cv::Mat::zeros(20, 20, CV_8UC1); 
  cv::rectangle( mat, cv::Point( 15, 0 ), cv::Point( 20, 20), 255, CV_FILLED);
  cv::rectangle( mat, cv::Point( 0, 17 ), cv::Point( 20, 20), 255, CV_FILLED);
  rc.SetMap(mat, 2.0, 0, 0);  // 2.0m per pixel

  // Test tracing from empty space into wall
  sensor_msgs::LaserScan s = rc.Scan(start, M_PI_2);  // Scan angled up

  // General properties
  EXPECT_NEAR(s.angle_min, -M_PI_2, 1e-5);
  EXPECT_EQ(3, s.ranges.size());

  // Right wall
  EXPECT_NEAR(s.ranges[0], 20.0, 1e-5);  // 20m to right wall
  EXPECT_NEAR(s.ranges[1], 24.0, 1e-5);  // 24m to top wall
  EXPECT_NEAR(s.ranges[2], 51.0, 1e-5);  // max range + 1m for no return
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
