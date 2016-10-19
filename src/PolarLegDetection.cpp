/*
 * Peforms leg detection using the polar form of the laser scanner data
 * As part of the overall goal of person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 12/10/2016
*/

#include <ros/ros.h>

#include <ass2/PolarLegDetection.hpp>

#define INF 1000.0
//#define SPIKE_DELTA 0.45 //was 0.9 but now trying to get it to work in more clustered environments

#define SPIKE_DISTANCE 0.4

#define SAME_LEG_DELTA 0.05
#define DETECTION_RADIUS 2.0

#define MIN_LEG_DISTANCE 0.045

// MAX_LEG_DISTANCE was 0.2 but increased when person moves sideways quickly leg gap increases and robot loses the person
#define MAX_LEG_DISTANCE 0.24

#define MIN_LEG_SIZE 8
#define LEG_SIZE_DELTA 12

using namespace std;

PolarLegDetector::PolarLegDetector() {
	ros::NodeHandle n;
	//create two empty leg vectors
	vector<crosbot::Point2D> leg1;
	vector<crosbot::Point2D> leg2;
	legs.push_back(leg1);
	legs.push_back(leg2);
    scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &PolarLegDetector::callbackScan, this);
	posePub = n.advertise<geometry_msgs::Pose>("legsCentre",10,false);
}



void PolarLegDetector::callbackScan(const sensor_msgs::LaserScanConstPtr& scan) {
	float angle = scan->angle_min;
	float prevValidRange = INF;
	float currRange;
	//float rangeDelta; //code now uses euclidean distance not range delta
	int legIndex = -1;
	crosbot::Point2D currPoint;
	crosbot::Point2D prevValidPoint = crosbot::Point2D(1000.0,1000.0);;
    for (int n = 0; n < scan->ranges.size(); ++n, angle += scan->angle_increment) {
		currRange = scan->ranges[n];
		cout << "Polar={"<<currRange<<","<<angle<<"} cartesian = ["<<cos(angle) * scan->ranges[n]<<","<< sin(angle) * scan->ranges[n]<<"]\n";
		if (currRange>0 && currRange<INF) {
			//rangeDelta = abs(prevValidRange-currRange);
			// valid point so create point
			currPoint = crosbot::Point2D(cos(angle)*currRange,sin(angle)*currRange);
			if (currRange<DETECTION_RADIUS) {
				// is a valid range
				// deal with changes like start of pair of legs, end of pair or change of leg
				if  (legIndex==0 && legs[0].size()>0 && legs[0][legs[0].size()-1].distanceTo(currPoint)>MIN_LEG_DISTANCE && legs[0][legs[0].size()-1].distanceTo(currPoint)<MAX_LEG_DISTANCE) {
					// switch legs
					if (debug) {
						cout << "Switching legs" << endl;
					}
					legs[1].clear();
					legIndex = 1;
				} else if (currRange<prevValidRange && prevValidPoint.distanceTo(currPoint)>SPIKE_DISTANCE) {
					//start of a pair of legs
					if (debug) {
						cout << "Start of a pair of legs" << endl;
					}
					legs[0].clear();
					legIndex = 0;
				} else if (legIndex==1 && prevValidPoint.distanceTo(currPoint)>MAX_LEG_DISTANCE && prevValidRange<DETECTION_RADIUS) {
					if (debug) {
						cout << "End of a pair of legs within detection radius\n";
					}
					//end of a pair of legs (possibly)
					if (legsDetected()) {
						//valid pair of legs detected
						if (debug) {
							cout << "Valid pair of legs detected\n";
						}
						break;
					} else {
						if (debug) {
							cout << "Invalid legs, reset\n";
						}
						//not actual legs, reset
						legs[0].clear();
						legs[1].clear();
						legIndex = 0;
					}
				}
				// this means that what fails as being the end of a pair of legs within detection radius
				// is considered to be the start of a pair of legs 
				legs[legIndex].push_back(currPoint);
			} else if (legIndex == 1 && prevValidRange<DETECTION_RADIUS && currRange>prevValidRange && prevValidPoint.distanceTo(currPoint)>SPIKE_DISTANCE) {
				//end of a pair of legs
				if (debug) {
					cout << "End of a pair of legs outside detection radius" << endl;
				}
				if (legsDetected()) {
					if (debug) {
						cout << "Valid pair of legs detected"<<endl;
					}
					// found a valid pair of legs
					break;
				} else {
					if (debug) {
						cout << "Invalid, reset"<<endl;
					}
					// not actual legs, reset
					legs[0].clear();
					legs[1].clear();
					legIndex = 0;
				}
			}
			//set previous valid range to current range
			prevValidRange = currRange;
			prevValidPoint = currPoint;
		}

	}

	if (legs[0].size()>0 && legs[1].size()>0 && legsDetected()) {
		if (debug) {
			cout << "No more valid points, valid pair of legs detected\n";
		}
	} else if (debug) {
		cout << "Final check for pair of legs failed, reset\n";
		legs[0].clear();
		legs[1].clear();
	}

	cout << "Legs centre at |" << legsCentre.x << "," << legsCentre.y << endl;
	cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n";

	debug = false;
	publishLegsCentre();
}



bool PolarLegDetector::legsDetected() {
	vector<crosbot::Point2D> leg1 = legs[0];
	vector<crosbot::Point2D> leg2 = legs[1];
	int leg1size = leg1.size();
	int leg2size = leg2.size();
	if (leg1size <=0 || leg2size<=0) {
		//avoid out of range errors
		return false;
	}
	if (debug) {
		cout << "Testing the following 2 leg vectors\n";
		printPoints(leg1);
		printPoints(leg2);
	}
	crosbot::Point2D p1 = leg1[leg1size-1];
	crosbot::Point2D p2 = leg2[0];
	if (leg1size>=MIN_LEG_SIZE && leg2size>=MIN_LEG_SIZE && abs(leg1size-leg2size)<=LEG_SIZE_DELTA && p1.distanceTo(p2)<=MAX_LEG_DISTANCE) {
		legsCentre = crosbot::Point2D((p1.x+p2.x)/2.0,(p1.y+p2.y)/2.0);
		return true;
	}
	return false;
}


void PolarLegDetector::printPoints(vector<crosbot::Point2D> points) {
	cout <<"[";
	for (int i=0; i<points.size(); i++) {
		crosbot::Point2D p = points[i];
		cout << "("<<p.x<<","<<p.y<<") ";
	}
	cout<<"]\n";
}


void PolarLegDetector::publishLegsCentre() {
	posePub.publish(crosbot::Pose3D(legsCentre.x,legsCentre.y,0).toROS());
}
