
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

beaconMarkers::beaconMarkers()
{

	time_t timer; //current time
	time(&start_time);
	time(&timer); 
      
    double diff = difftime(timer, start_time);

    if (diff < (double) 5) {
      std::cout << "diff was 2 small\n";
		  return;
    }
    std::cout << "diff was not too small\n";

    beacon_sub = nh_.subscribe("/beaconMessage", 100, &beaconMarkers::beacon_callback, this);

    marker_pub = nh_.advertise<visualization_msgs::Marker>("/beacons", 1);

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
      std::cout << "YOU FUCKED UP LOL\n";
    	return;
    }
    
    std::cout << "you didn't fuck up lol\n";
    
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
    
    std::cout << "put a marker of colour " << marker.ns << " at position " << marker.pose.position.x << " " <<  marker.pose.position.y << " " << marker.pose.position.z << "\n";
    marker_pub.publish(marker);
}

void beaconMarkers::beacon_callback(const ass2::beacon_msg &beacon_msg) {
	beacon = beacon_msg;
}

