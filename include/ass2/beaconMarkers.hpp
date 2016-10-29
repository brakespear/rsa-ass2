#ifndef BEACONMARKERS_H
#define BEACONMARKERS_H

#include <ros/ros.h>
#include "ass2/image_converter.hpp"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <cmath>

//centremost column of beacon detection image, used to calculate how close a beacon is to the centre
#define CENTRE_COLUMN 317

class beaconMarkers {
private:
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener;
	time_t start_time;

	ros::Subscriber beacon_sub;
	ros::Publisher marker_pub;

	//number to detect how close the detected beacons are to the centre. closer to 0 means 
	//closer to centre. 1000 is default number so that the detected beacon difference will 
	//always be smaller (under 650 or so) so that the first time the beacon is detected it will 
	//always be a smaller number and be reset	
	int yellowPinkMostCentre = 1000;
	int pinkYellowMostCentre = 1000;
	int bluePinkMostCentre = 1110;
	int pinkGreenMostCentre = 1000;
	

public:
	//described in cpp file
	beaconMarkers();
	void beacon_callback(const ass2::beacon_msg &beacon_msg);
};

#endif
