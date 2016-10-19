#ifndef BEACONMARKERS_H
#define BEACONMARKERS_H

#include <ros/ros.h>
#include "ass2/image_converter.hpp"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <cmath>

#define PI 3.14159265

class beaconMarkers {
private:
	ros::NodeHandle nh_;
	//ass2::beacon_msg beacon;
	//visualization_msgs::Marker marker;
	tf::TransformListener tf_listener;
	//tf::StampedTransform transform;
	time_t start_time;

	ros::Subscriber beacon_sub;
	ros::Publisher marker_pub;
	
	double calculateAngle(int column);

public:
	beaconMarkers();
	void beacon_callback(const ass2::beacon_msg &beacon_msg);
};

#endif
