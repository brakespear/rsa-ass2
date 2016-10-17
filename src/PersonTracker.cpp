/*
 * Implements the person tracker class
 * Which navigates the robot to the legs centre found by the leg detection code
 * In order to achieve person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 15/10/2015
*/

#include <ros/ros.h>

#include <ass2/PersonTracker.hpp>

#define MAX_SPEED 0.2

PersonTracker::PersonTracker() {
	ros::NodeHandle n;
	poseSub = n.subscribe<geometry_msgs::Pose>("legsCentre",1,&PersonTracker::legsCentrePoseCb,this);
	drivePub = n.advertise< geometry_msgs::Twist >("cmd_vel_mux/input/navi", 1, false);
	markerPub = n.advertise< visualization_msgs::Marker >("destination",1,false);
}

void PersonTracker::legsCentrePoseCb(const geometry_msgs::PoseConstPtr& legsCentrePose) {
	float centre_x = legsCentrePose->position.x;
	float centre_y = legsCentrePose->position.y;
	//the y coordinate determines how much to turn left (+ve y) or right (-ve y)
	geometry_msgs::Twist t;
	t.linear.y = t.linear.z = 0;
	t.linear.x = sqrt((centre_x)/4.0) *MAX_SPEED;
	t.angular.x = t.angular.y = 0;
	t.angular.z = centre_y; //might need to invert the sign
	publishMarker(*legsCentrePose);
	drivePub.publish(t);
}

void PersonTracker::publishMarker(const geometry_msgs::Pose legsCentre) {
	visualization_msgs::Marker dest;
	dest.header.stamp = ros::Time::now();
	dest.header.frame_id = "base_link"; //this might need to be changed
	dest.id = 5;
	dest.type = 2; // sphere
	dest.action = 0; // add/modify
	dest.pose = legsCentre;
	dest.color.r = 1.0;
	dest.color.g = dest.color.b = 0.0;
	dest.lifetime = ros::Duration(0);
	markerPub.publish(dest);
}


