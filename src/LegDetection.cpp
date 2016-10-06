/*
 * Peforms leg detection
 * As part of the overall goal of person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 5/10/2016
*/

#include <ros/ros.h>

#include <ass2/LegDetection.hpp>

using namespace std;

LegDetector::LegDetector() {
	ros::NodeHandle n;
    scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &LegDetector::callbackScan, this);
}

void LegDetector::callbackScan(const sensor_msgs::LaserScanConstPtr& scan) {
    tf::StampedTransform transform;
    // get laser to base_link transform
    try {
        tfListener.waitForTransform(BASE_FRAME, scan->header.frame_id, scan->header.stamp, ros::Duration(2.0));
        tfListener.lookupTransform(BASE_FRAME, scan->header.frame_id, scan->header.stamp, transform);
    } catch (tf::TransformException& tfe) {
        ROS_ERROR("Unable to get transformation.");
        return;
    }


    // Turn laser scan into a point cloud
    pointCloud.resize(scan->ranges.size());
    float angle = scan->angle_min;
    for (int n = 0; n < pointCloud.size(); ++n, angle += scan->angle_increment) {
	    // Polar to cartesian co-ordinates
        tf::Vector3 point(cos(angle) * scan->ranges[n], sin(angle) * scan->ranges[n], 0);

        // transfer point to base_link frame
        point = transform * point;
		// create 2D crosbot point
		crosbotPoint = Point2D(point.x(),point.y());
		// add to point cloud
		pointCloud.push_back(crosbotPoint);
    }
	
	
}





