#include <ros/ros.h>

#include <cstdio>
#include <cstdlib>


#include <ass2/PolarLegDetection.hpp>
#include <ass2/PersonTracker.hpp>


#define LOG_START  "Ass2PersonFollowingNode ::"

int main(int argc, char **argv) {
	ros::init(argc, argv, "person_tracking_node");

	ros::NodeHandle nh("~");
	PolarLegDetector polarLegDetector;
	PersonTracker personTracker;
	while(ros::ok()) {
		ROS_INFO("%s spinning",LOG_START);
		ros::spin();
	}

	return 0;
}

