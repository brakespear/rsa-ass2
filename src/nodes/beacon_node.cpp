#include <ros/ros.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#include <ass2/beacon.hpp>
#include <ass2/image_converter.hpp>

#define LOG_START  "Ass2BeaconNode ::"

int main(int argc, char **argv) {
	ros::init(argc, argv, "beacon_node");

	ros::NodeHandle nh("~");
  ImageConverter ic;
  //ROS_INFO("hello there\n");
	
	//std::vector<Beacon> beacons = Beacon::parseBeacons(paramNh);
	
	while(ros::ok()) {
		//ROS_INFO("%s spinning",LOG_START);
		ros::spin();
	}

	return 0;
}
