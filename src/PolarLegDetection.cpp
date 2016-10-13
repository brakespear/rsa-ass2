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
#define SPIKE_DELTA 2.0
#define SAME_LEG_DELTA 0.005
#define DETECTION_RADIUS 2.5
#define CHANGE_LEG_DELTA 0.2
#define MAX_LEG_GAP 0.2
#define MIN_LEG_SIZE 8
#define LEG_SIZE_DELTA 4

using namespace std;

PolarLegDetector::PolarLegDetector() {
	ros::NodeHandle n;
	//create two empty leg vectors
	vector<crosbot::Point2D> leg1;
	vector<crosbot::Point2D> leg2;
	legs.push_back(leg1);
	legs.push_back(leg2);
    scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &PolarLegDetector::callbackScan, this);

}



void PolarLegDetector::callbackScan(const sensor_msgs::LaserScanConstPtr& scan) {
	float angle = scan->angle_min;
	float prevValidRange = INF;
	float currRange;
	float rangeDelta;
	int legIndex = 0;
	crosbot::Point2D currPoint;
	vector<crosbot::Point2D> currLeg;
    for (int n = 0; n < scan->ranges.size(); ++n, angle += scan->angle_increment) {
		currRange = scan->ranges[n];
		if (!isnan(currRange)) {
			cout << "Valid point" << endl;
			rangeDelta = abs(prevValidRange-currRange);
			if (currRange<DETECTION_RADIUS) {
				cout << "Within detection radius" << endl;
				// is a valid range
				// so create point
				currPoint = crosbot::Point2D(cos(angle)*currRange,sin(angle)*currRange);
				//rangeDelta = abs(prevValidRange-currRange);
				// deal with changes like start of pair of legs, end of pair or change of leg
				if (currRange<prevValidRange && rangeDelta>=SPIKE_DELTA) {
					//start of a pair of legs
					cout << "Start of a pair of legs" << endl;
					legs[0].clear();
					legIndex = 0;
				} else if (rangeDelta>SAME_LEG_DELTA && rangeDelta<=CHANGE_LEG_DELTA) {
					// switch legs
					cout << "Switching legs" << endl;
					legs[1].clear();
					legIndex = 1;
				}
				cout << "Push point to leg" << endl;
				//now leg index is set correctly
				currLeg = legs[legIndex];
				currLeg.push_back(currPoint);
			} else if (prevValidRange<DETECTION_RADIUS && currRange>prevValidRange && rangeDelta>=SPIKE_DELTA) {
				//end of a pair of legs
				cout << "End of a pair of legs" << endl;
				if (legsDetected()) {
					cout << "Valid pair of legs detected"<<endl;
					// found a valid pair of legs
					break;
				} else {
					cout << "Invalid, reset"<<endl;
					// not actual legs, reset
					legs[0].clear();
					legs[1].clear();
					legIndex = 0;
				}
			}
			//set previous valid range to current range
			prevValidRange = currRange;
		}



		if (debug) {
			cout << "Polar={"<<currRange<<","<<angle<<"} cartesian = ["<<cos(angle) * scan->ranges[n]<<","<< sin(angle) * scan->ranges[n]<<"]\n";
			vector<crosbot::Point2D> leg1 = legs[0];
			vector<crosbot::Point2D> leg2 = legs[1];
			if (debug) {
				cout << "leg vectors are:"<<endl;
				printPoints(leg1);
				printPoints(leg2);
			}
		}
	}
	if (debug) {
		cout << "Legs centre at " << legsCentre.x << "," << legsCentre.y << endl;
	}
	debug = false;
}



bool PolarLegDetector::legsDetected() {
	vector<crosbot::Point2D> leg1 = legs[0];
	vector<crosbot::Point2D> leg2 = legs[1];
	if (debug) {
		cout << "Testing the following 2 leg point vectors for a pair of legs:"<<endl;
		printPoints(leg1);
		printPoints(leg2);
	}
	cout << "here"<<endl;
	int leg1size = leg1.size();
	int leg2size = leg2.size();
	cout << "leg1 has size"<<leg1size<<endl;
	crosbot::Point2D p1 = leg1[leg1size-1];
	crosbot::Point2D p2 = leg2[0];
	if (leg1size>=MIN_LEG_SIZE && leg2size>=MIN_LEG_SIZE && abs(leg1size-leg2size)<=LEG_SIZE_DELTA && p1.distanceTo(p2)<=MAX_LEG_GAP) {
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
