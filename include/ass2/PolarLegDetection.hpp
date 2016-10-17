/*
 * Defines the PolarLegDetection class
 * To be used to achieve leg detection as part of the overall goal of person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 12/10/2016
*/

#ifndef POLAR_LEG_DETECTION_HPP
#define POLAR_LEG_DETECTION_HPP

#include <crosbot/handle.hpp>
#include <crosbot/geometry/points.hpp>
#include <crosbot/geometry/poses.hpp>

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>

#include <sstream>

using namespace std;

class PolarLegDetector {
private:
	ros::Subscriber scanSub;
	ros::Publisher posePub;
	int debug = true;
	vector<vector<crosbot::Point2D>> legs;
	crosbot::Point2D legsCentre;
	
public:
	PolarLegDetector();
	virtual ~PolarLegDetector() {};
	void callbackScan(const sensor_msgs::LaserScanConstPtr& scan);
	bool legsDetected();
	void printPoints(vector<crosbot::Point2D> points);
	void publishLegsCentre();

};




#endif
