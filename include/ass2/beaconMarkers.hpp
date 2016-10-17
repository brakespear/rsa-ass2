#ifndef BEACONMARKERS_H
#define BEACONMARKERS_H

#include <ros/ros.h>
#include "ass2/image_converter.hpp"

class beaconMarkers {
private:
	ros::NodeHandle nh_;
	ass2::beacon_msg beacon;

	ros::Subscriber beacon_sub;
	ros::Publisher marker_pub;

public:
	publishBeacon();
	void beacon_callback(const ass2::beacon_msg &beacon_msg);
};

#endif