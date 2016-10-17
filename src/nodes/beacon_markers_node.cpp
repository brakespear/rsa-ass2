#include <ros/ros.h>

#include <cstdio>
#include <cstdlib>

#include <ass2/beaconMarkers.hpp>

#define LOG_START  "Ass2BeaconMarkersNode ::"

int main(int argc, char **argv) {
	ros::init(argc, argv, "beacon_markers_node");

	ros::NodeHandle nh("~");
  beaconMarkers bm;
	
	while(ros::ok()) {
		//ROS_INFO("%s spinning",LOG_START);
		ros::spin();
	}

	return 0;
}
