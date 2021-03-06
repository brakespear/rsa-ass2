/*
 * Defines the LegDetection class
 * To be used to achieve leg detection as part of the overall goal of person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 5/10/2016
*/

#ifndef LEG_DETECTION_HPP
#define LEG_DETECTION_HPP

#include <crosbot/handle.hpp>
#include <crosbot/geometry/points.hpp>

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>

#include <sstream>

using namespace std;

class LegDetector {
private:
	ros::Subscriber scanSub;
	//tf::TransformListener tfListener;
	
	vector<crosbot::Point2D> pointCloud;
	vector<vector<crosbot::Point2D>> clusters;
	int debug = true;
	crosbot::Point2D legsCenter;
	
public:
	LegDetector();
	virtual ~LegDetector() {};
	void callbackScan(const sensor_msgs::LaserScanConstPtr& scan);
	void printClusters();
	void findClusters();
	void printPointCloud();
	void findLegs();
	bool legPair(vector<crosbot::Point2D> cluster1, vector<crosbot::Point2D> cluster2);
	bool singleLegCluster(vector<crosbot::Point2D> cluster);
	void printCluster(vector<crosbot::Point2D> cluster);
	double calculateGradient(crosbot::Point2D p1, crosbot::Point2D p2);
};




#endif


