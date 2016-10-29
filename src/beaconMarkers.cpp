
#include <ros/ros.h>
#include "ass2/beaconMarkers.hpp"

//using namespace cv;

beaconMarkers::beaconMarkers()
{
  beacon_sub = nh_.subscribe("/beaconMessage", 100, &beaconMarkers::beacon_callback, this);

  marker_pub = nh_.advertise<visualization_msgs::Marker>("/beacons", 1);
  
  time(&start_time);
	
}

/*
 * Callback for when a beacon is detected. Determines which beacon was detected, and if that beacon
 * is closer to the centre than the current detection being used for that beacon, this beacon will
 * be placed on the map and erase the old one. The marker colours and size are set and sent to the
 * map. The position, orientation and distance of the beacon from the robot (depth) is used to 
 * determine where to place the beacon on the map. 
 */
void beaconMarkers::beacon_callback(const ass2::beacon_msg &beacon_msg) {
	
	time_t timer; //current time
	time(&timer); 
      
    double diff = difftime(timer, start_time);

    //only detect beacon if > 5 seconds have passed since program started running
    if (diff < (double) 5) {
		  return;
    }
    
    tf::StampedTransform transform;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker topmarker;
    

    marker.header.frame_id="map";
    marker.header.stamp = ros::Time::now();
    
    topmarker.header.frame_id="map";
    topmarker.header.stamp = ros::Time::now();
    
    if (beacon_msg.bottomColour == "pink" && beacon_msg.topColour == "yellow") {
	    marker.ns = "pink";
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.75;

	    topmarker.ns = "yellow";
	    topmarker.color.r = 1.0;
	    topmarker.color.g = 1.0;
	    topmarker.color.b = 0.0;
	    
	    int diff = abs(CENTRE_COLUMN - beacon_msg.col);
	    if (diff > yellowPinkMostCentre) {
	      return;
	    } else {
	      yellowPinkMostCentre = diff;
	    }

    } else if (beacon_msg.bottomColour == "green" && beacon_msg.topColour == "pink") {
	    marker.ns = "green";
	    marker.color.r =0.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0.0;

	    topmarker.ns = "pink";
	    topmarker.color.r = 1.0;
	    topmarker.color.g = 0.0;
	    topmarker.color.b = 0.75;
	    
	    //check where beacon is - if more centre than current detection replace it
	    int diff = abs(CENTRE_COLUMN - beacon_msg.col);
	    if (diff > pinkGreenMostCentre) {
	      return;
	    } else {
	      pinkGreenMostCentre = diff;
	    }

    } else if (beacon_msg.bottomColour == "yellow" && beacon_msg.topColour == "pink") {
	    marker.ns = "yellow";
	    marker.color.r =1.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0.0;

	    topmarker.ns = "pink";
	    topmarker.color.r = 1.0;
	    topmarker.color.g = 0.0;
	    topmarker.color.b = 0.75;
	    
	    int diff = abs(CENTRE_COLUMN - beacon_msg.col);
	    if (diff > pinkYellowMostCentre) {
	      return;
	    } else {
	      pinkYellowMostCentre = diff;
	    }

    }else if (beacon_msg.bottomColour == "pink" && beacon_msg.topColour == "blue") {
	    marker.ns = "pink";
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.75;

	    topmarker.ns = "blue";
	    topmarker.color.r = 0.0;
	    topmarker.color.g = 0.0;
	    topmarker.color.b = 1.0;
	    
	    int diff = abs(CENTRE_COLUMN - beacon_msg.col);
	    if (diff > bluePinkMostCentre) {
	      return;
	    } else {
	      bluePinkMostCentre = diff;
	    }

    } else {
      std::cout << "invalid beacon\n";
    	return;
    }
    
    marker.id = beacon_msg.id;
    topmarker.id = beacon_msg.id + 4;
    
    try 
    {
      tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

      //determine angles by finding cos/sin of current orientation of the robot
      double cosAngle = cos(tf::getYaw(transform.getRotation()));
      double sinAngle = sin(tf::getYaw(transform.getRotation())); 
		  
      //use angles and depth to determine the position of the beacin
      marker.pose.position.x = transform.getOrigin().x() + beacon_msg.depth * cosAngle;     
      marker.pose.position.y = transform.getOrigin().y() + beacon_msg.depth * sinAngle;
      marker.pose.position.z = 0.2;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      topmarker.pose.position.x = transform.getOrigin().x() + beacon_msg.depth * cosAngle;
      topmarker.pose.position.y = transform.getOrigin().y() + beacon_msg.depth * sinAngle;
      topmarker.pose.position.z = 0.5;
      topmarker.pose.orientation.x = 0.0;
      topmarker.pose.orientation.y = 0.0;
      topmarker.pose.orientation.z = 0.0;
      topmarker.pose.orientation.w = 1.0;
    }
    catch (tf::TransformException ex) {
    	ROS_ERROR("Nope! %s\n", ex.what());
    }
    
    //add two markers; one for bottom colour and one for top colour
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 0.3; //0.1;
    marker.scale.y = 0.3; //0.1;
    marker.scale.z = 0.3; //0.1;
    marker.color.a = 1;

    marker.frame_locked = true;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    
    std::cout << "put a marker of colour " << marker.ns << " at position " << marker.pose.position.x
            << " " <<  marker.pose.position.y << " " << marker.pose.position.z << "\n";
    marker_pub.publish(marker);

    topmarker.type = visualization_msgs::Marker::CYLINDER;
    topmarker.scale.x = 0.3; //0.1;
    topmarker.scale.y = 0.3; //0.1;
    topmarker.scale.z = 0.3; //0.1;
    topmarker.color.a = 1;
    
    topmarker.frame_locked = true;
    topmarker.lifetime = ros::Duration();
    topmarker.action = visualization_msgs::Marker::ADD;
    
    std::cout << "put a top marker of colour " << topmarker.ns << " at position " << 
            topmarker.pose.position.x << " " <<  topmarker.pose.position.y << " " << 
            topmarker.pose.position.z << "\n";
    marker_pub.publish(topmarker);
}
