
#include <ros/ros.h>
#include "ass2/beaconMarkers.hpp"

/*
beacon_msg.msg:
int8 id
int32 row
int32 col
float64 depth
string topColour
string bottomColour
int32 minRow
int32 maxRow
*/

using namespace cv;

void beaconMarkers::publishBeacon() {
    //tf::StampedTransform transform;
	time_t timer, start_time; //current time
	time(&timer); 
      
    double diff = difftime(timer, start_time);
    if (diff < (double) 5) {
		return;
    }
    beacon_sub = nh_.subscribe("/beaconMessage", 100, &callback, this);

    marker_pub = nh_.advertise<visualization_msgs::Marker>("/beacons", 1);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id="base_link";
    marker.header.stamp = ros::Time();
    
    if (beacon.bottomColour == "pink" && beacon.topColour == "yellow") {
	    marker.id = beacon.id;
	    marker.ns = "pink";
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.75;
    	time(&start_time);

    } else if (beacon.bottomColour == "green") {
	    marker.id = beacon.id;
	    marker.ns = "green";
	    marker.color.r =0.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0.0;
    	time(&start_time);

    } else if (beacon.bottomColour == "yellow") {
	    marker.id = beacon.id;
	    marker.ns = "yellow";
	    marker.color.r =0.0;
	    marker.color.g = 1.0;
	    marker.color.b = 1.0;
	    time(&start_time);

    }else if (beacon.topColour == "blue") {
	    marker.id = beacon.id;
	    marker.ns = "blue";
	    marker.color.r = 0.0;
	    marker.color.g = 0.0;
	    marker.color.b = 1.0;
	    time(&start_time);

    } else {
    	return;
    }
    
    try 
    {
		tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

		marker.pose.position.x = beacon.col;
		marker.pose.position.y = beacon.row;
		marker.pose.position.z = beacon.depth;
    }
    catch (tf::TransformException ex) {
    	ROS_ERROR("Nope! %s\n", ex.what());
    }
    
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1;

    
    
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    
    marker_pub.publish(marker);
}

void beaconMarkers::beacon_callback(const ass2::beacon_msg &beacon_msg) {
	beacon = beacon_msg;
	/*
	int id = beacon_msg->id;
	int row = beacon_msg->row;
	int col = beacon_msg->col;
	float depth = beacon_msg->depth;
	std::string topColour = beacon_msg->topColour;
	std::string bottomColour = beacon_msg->bottomColour;
	int minRow = beacon_msg->minRow;
	int maxRow = beacon_msg->maxRow;
	*/
}


