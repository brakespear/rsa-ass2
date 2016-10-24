#ifndef BEACONMARKERS_H
#define BEACONMARKERS_H

#include <ros/ros.h>
#include "ass2/image_converter.hpp"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <cmath>

#define PI 3.14159265
#define CENTRE_COLUMN 317

class beaconMarkers {
private:
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener;
	time_t start_time;

	ros::Subscriber beacon_sub;
	ros::Publisher marker_pub;
	
	int yellowPinkMostCentre = 1000;
	int pinkYellowMostCentre = 1000;
	int bluePinkMostCentre = 1110;
	int pinkGreenMostCentre = 1000;
	

public:
	beaconMarkers();
	void beacon_callback(const ass2::beacon_msg &beacon_msg);
};

#endif
