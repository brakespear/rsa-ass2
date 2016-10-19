
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

//using namespace cv;

beaconMarkers::beaconMarkers()
{
  beacon_sub = nh_.subscribe("/beaconMessage", 100, &beaconMarkers::beacon_callback, this);

  marker_pub = nh_.advertise<visualization_msgs::Marker>("/beacons", 1);
  
  time(&start_time);
  //std::cout << "start time is " << start_time << "\n";
	
}

void beaconMarkers::beacon_callback(const ass2::beacon_msg &beacon_msg) {
	
	time_t timer; //current time
	time(&timer); 
	std::cout << "current time is " << timer << " and start time is " << start_time << "\n";
      
    double diff = difftime(timer, start_time);

    if (diff < (double) 5) {
      std::cout << "diff was 2 small, it is " << diff << "\n";
		  return;
    }
    std::cout << "diff was not too small\n";
    
    tf::StampedTransform transform;
    visualization_msgs::Marker marker;

    

    marker.header.frame_id="/map";
    marker.header.stamp = ros::Time::now();
    
    if (beacon_msg.bottomColour == "pink" && beacon_msg.topColour == "yellow") {
	    marker.id = beacon_msg.id;
	    marker.ns = "pink";
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.75;
    	//time(&start_time);

    } else if (beacon_msg.bottomColour == "green") {
	    marker.id = beacon_msg.id;
	    marker.ns = "green";
	    marker.color.r =0.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0.0;
    	//time(&start_time);

    } else if (beacon_msg.bottomColour == "yellow") {
	    marker.id = beacon_msg.id;
	    marker.ns = "yellow";
	    marker.color.r =0.0;
	    marker.color.g = 1.0;
	    marker.color.b = 1.0;
	    //time(&start_time);

    }else if (beacon_msg.topColour == "blue") {
	    marker.id = beacon_msg.id;
	    marker.ns = "blue";
	    marker.color.r = 0.0;
	    marker.color.g = 0.0;
	    marker.color.b = 1.0;
	    //time(&start_time);

    } else {
      std::cout << "YOU FUCKED UP LOL\n";
    	return;
    }
    
    std::cout << "you didn't fuck up lol\n";
    
    try 
    {
		  tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		  std::cout << "transform worked\n";

      double angle = cos(transform.getRotation().getAngle());
      std::cout << "calculated cos angle is " << angle << "\n";

		  marker.pose.position.x = transform.getOrigin().x() + beacon_msg.depth * angle;
		  
		  angle = sin(transform.getRotation().getAngle());
      std::cout << "calculated sin angle is " << angle << "\n";
		  
      marker.pose.position.y = transform.getOrigin().y() + beacon_msg.depth * angle;
      marker.pose.position.z = transform.getOrigin().z();
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
    }
    catch (tf::TransformException ex) {
    	ROS_ERROR("Nope! %s\n", ex.what());
    }
    
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 0.3; //0.1;
    marker.scale.y = 0.3; //0.1;
    marker.scale.z = 0.3; //0.1;

    marker.color.a = 1;

    
    marker.frame_locked = true;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    
    std::cout << "put a marker of colour " << marker.ns << " at position " << marker.pose.position.x << " " <<  marker.pose.position.y << " " << marker.pose.position.z << "\n";
    marker_pub.publish(marker);
}

double beaconMarkers::calculateAngle(int column) {
  double ret = column/(500.0/180.0) - 90.0;
  std::cout << "raw angle is " << ret << "\n";
  return ret;
}

