/*
 * Peforms leg detection
 * As part of the overall goal of person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 5/10/2016
*/

#include <ros/ros.h>

#include <ass2/LegDetection.hpp>

#define MIN_RANGE 0.0
#define MAX_RANGE 10.0
#define CLUSTER_DISTANCE 0.025

using namespace std;

LegDetector::LegDetector() {
	ros::NodeHandle n;
    scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &LegDetector::callbackScan, this);

}



void LegDetector::callbackScan(const sensor_msgs::LaserScanConstPtr& scan) {
    // Turn laser scan into a point cloud 
	float angle = scan->angle_min;
    for (int n = 0; n < scan->ranges.size(); ++n, angle += scan->angle_increment) {
		if (scan->ranges[n]>MIN_RANGE && scan->ranges[n]<MAX_RANGE) {
			// create 2D crosbot point
			crosbot::Point2D crosbotPoint = crosbot::Point2D(cos(angle) * scan->ranges[n],sin(angle) * scan->ranges[n]);
			// add to point cloud
			pointCloud.push_back(crosbotPoint);
		}
    }
	//find clusters
	findClusters();
	if (debug) {
		//printPointCloud();
		printClusters();
		debug = false;
	}
	
	
}

void LegDetector::findClusters() {
	clusters.clear();
	crosbot::Point2D prevPoint = pointCloud[0];
	crosbot::Point2D currPoint;
	vector<crosbot::Point2D> currentCluster;
	currentCluster.push_back(prevPoint);
	for (int i=1; i<pointCloud.size(); i++) {
		currPoint = pointCloud[i];
		if (prevPoint.distanceTo(currPoint)>CLUSTER_DISTANCE) {
			clusters.push_back(currentCluster);
			currentCluster.clear();
		}
		currentCluster.push_back(currPoint);
		prevPoint = currPoint;
	}
	clusters.push_back(currentCluster);
}
			

void LegDetector::printClusters() {
	crosbot::Point2D currPoint;
	for (int i=0; i<clusters.size(); i++) {
		vector<crosbot::Point2D> currCluster = clusters[i];
		cout << "[";
		for (int j=0; j<currCluster.size(); j++) {
			currPoint = currCluster[j];
			cout << "(" << currPoint.x << "," << currPoint.y << ") ";
		}
		cout << "]\n";
	}
}

void LegDetector::printPointCloud() {
	crosbot::Point2D currPoint;
	for (int i=0; i<pointCloud.size();i++) {
		currPoint = pointCloud[i];
		cout << "("<<currPoint.x<<","<<currPoint.y<<")\n";
	}
}

	


