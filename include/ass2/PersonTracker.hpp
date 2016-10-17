/*
 * Defines the person tracker class
 * Which navigates the robot to the legs centre found by the leg detection code
 * In order to achieve person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 15/10/2015
*/

#ifndef PERSON_TRACKER_HPP
#define PERSON_TRACKER_HPP


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


class PersonTracker{
private:
	ros::Subscriber poseSub;
	ros::Publisher drivePub;
	ros::Publisher markerPub;
public:
	PersonTracker();
	virtual ~PersonTracker() {};
	void legsCentrePoseCb(const geometry_msgs::PoseConstPtr& legsCentrePose);
	void publishMarker(const geometry_msgs::Pose legsCentre);
};

#endif


