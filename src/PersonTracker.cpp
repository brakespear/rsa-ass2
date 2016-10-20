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

#define MAX_SPEED 0.8 //0.5
#define MAX_TURN 3


PersonTracker::PersonTracker() {
	ros::NodeHandle n;
	poseSub = n.subscribe<geometry_msgs::Pose>("legsCentre",1,&PersonTracker::legsCentrePoseCb,this);
	drivePub = n.advertise< geometry_msgs::Twist >("cmd_vel_mux/input/navi", 1, false);
}

void PersonTracker::legsCentrePoseCb(const geometry_msgs::PoseConstPtr& legsCentrePose) {
	float centre_x = legsCentrePose->position.x;
	float centre_y = legsCentrePose->position.y;
	ROS_INFO("Legs centre at (%.2f,%.2f)\n",centre_x,centre_y);
	//the y coordinate determines how much to turn left (+ve y) or right (-ve y)
	geometry_msgs::Twist t;
	t.linear.y = t.linear.z = 0;
	// stop the robot when it catches up to the person
	if (centre_x<0.55) {
		t.linear.x = 0;
		t.angular.z = 0;
	} else {
		t.linear.x = sqrt((centre_x)/4.0) *MAX_SPEED;
		t.angular.z = centre_y * MAX_TURN;
	}
	t.angular.x = t.angular.y = 0;
	// restrict the max turn speed to avoid wild spins
	if (t.angular.z>2) {
		t.angular.z = 2.0;
	} else if (t.angular.z<-2.0) {
		t.angular.z = -2.0;
	}
	ROS_INFO("Publishing velocity %.2f forward %.2f turn\n",t.linear.x,t.angular.z);
	drivePub.publish(t);
}


