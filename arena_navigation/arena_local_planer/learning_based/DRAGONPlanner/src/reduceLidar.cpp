#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher laserPub;
float cap;

void lasercb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	sensor_msgs::LaserScan pubMsg;
	pubMsg.header = msg->header;
	pubMsg.angle_min = msg->angle_min;
	pubMsg.angle_max = msg->angle_max;
	pubMsg.angle_increment = msg->angle_increment;
	pubMsg.time_increment = msg->time_increment;
	pubMsg.scan_time = msg->scan_time;
	pubMsg.range_min = msg->range_min;
	pubMsg.range_max = cap;

	pubMsg.ranges.resize(msg->ranges.size());

	for (size_t i = 0; i < msg->ranges.size(); i++)
	{
		if (!std::isinf(msg->ranges[i]) && !std::isnan(msg->ranges[i]))
			pubMsg.ranges[i] = std::min(msg->ranges[i], cap);
		else
			pubMsg.ranges[i] = msg->ranges[i];
	}

	laserPub.publish(pubMsg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "truncate_lidar");
	ros::NodeHandle nh;

	nh.getParam("reduce_lidar/cap", cap);

	ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &lasercb);
	laserPub = nh.advertise<sensor_msgs::LaserScan>("/truncate/scan", 10);

	ros::spin();
}
