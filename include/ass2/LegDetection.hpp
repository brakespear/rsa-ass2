/*
 * Defines the LegDetection class
 * To be used to achieve leg detection as part of the overall goal of person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 5/10/2016
*/

#ifndef LEG_DETECTION_HPP
#define LEG_DETECTION_HPP


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


#include <tf/transform_listener.h>
#include <crosbot/geometry/points.hpp>

using namespace std;

class LegDetector {
private:
	ros::Subscriber scanSub;
	tf::TransformListener tfListener;
	
	list<Point2D> pointCloud;
	list<list<Point2D>> clusters;
	int numClusters;
	
public:
	LegDetector();
	virtual ~LegDetector() {};
	void callbackScan(const sensor_msgs::LaserScanConstPtr& scan);

};




#endif


