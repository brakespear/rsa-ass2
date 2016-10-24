/*
 * Peforms leg detection using the polar form of the laser scanner data
 * As part of the overall goal of person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 12/10/2016
*/

#include <ros/ros.h>

#include <ass2/PolarLegDetection.hpp>

#define INF 1000.0 //range for inf
#define SPIKE_DISTANCE 0.4 //min euclidean distance between two points that form the boundary of a pair of legs

#define DETECTION_RADIUS 2.0 //Range which person must be inside in order to be detected/followed

#define MIN_LEG_DISTANCE 0.045 //Minimum euclidean distance between two legs
#define MAX_LEG_DISTANCE 0.24  //Maximum euclidean distance between two legs

#define MIN_LEG_SIZE 8 //Minimum number of points in a cluster to form a leg
#define LEG_SIZE_DELTA 12 //Max allowable different in the number of points that form the two leg clusters

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


/*
 * Search the laser scan for 2 cylinders that comprise a pair of legs
 * Looks for the first pair of legs it sees scanning anticlockwise from the right hand side
 * Uses both the polar coordinate frame and the cartesian plane to formula conditions for the 
 * Start/end/gap between legs
 * Only detects legs within a certain range
 * Then publishes the centre of the legs to be used by the follower
 * If no legs were detected in the current scan this legs centre point will be the last detected position
 * Giving the robot the behaviour of going to the last known position of the person when it fails to detect 
 * them again
*/
void PolarLegDetector::callbackScan(const sensor_msgs::LaserScanConstPtr& scan) {
	float angle = scan->angle_min;
	float prevValidRange = INF;
	float currRange;
	int legIndex = 0; //0 if on leg 1, 1 if on leg 2
	crosbot::Point2D currPoint;
    // create previous valid point a long way away to allow for immediate detection of the start of
    // a pair of legs
	crosbot::Point2D prevValidPoint = crosbot::Point2D(1000.0,1000.0);;
    for (int n = 0; n < scan->ranges.size(); ++n, angle += scan->angle_increment) {
		currRange = scan->ranges[n];
        // OUTPUT FOR PYTHON SIMULATION 
		//cout << "Polar={"<<currRange<<","<<angle<<"} cartesian = ["<<cos(angle) * scan->ranges[n]<<","<< sin(angle) * scan->ranges[n]<<"]\n";
		if (currRange>0 && currRange<INF) {
			// valid point so create point
            // polar to cartesian coordinates
			currPoint = crosbot::Point2D(cos(angle)*currRange,sin(angle)*currRange);
			if (currRange<DETECTION_RADIUS) {
				// Within detection radius, so point might be part of a leg
				// deal with changes like start of pair of legs, end of pair or change of leg
                
                // If currently on leg1, and leg1 has at least one point, and the distance between the last point of leg1 to this point is between the required values, then have detected the start of the second leg
				if  (legIndex==0 && legs[0].size()>0 && legs[0][legs[0].size()-1].distanceTo(currPoint)>MIN_LEG_DISTANCE && legs[0][legs[0].size()-1].distanceTo(currPoint)<MAX_LEG_DISTANCE) {
					// switch legs
					if (debug) {
						cout << "Switching legs" << endl;
					}
					legs[1].clear();
					legIndex = 1;
                // if current range is less than previous and distance between the two points is large
				} else if (currRange<prevValidRange && prevValidPoint.distanceTo(currPoint)>SPIKE_DISTANCE) {
					//start of a pair of legs
					if (debug) {
						cout << "Start of a pair of legs" << endl;
					}
					legs[0].clear();
					legIndex = 0;
                // if on leg2, distance between the two points too large to be the gap between 2 legs
                // and the previous range was within detection radius
                // then the previous point might have been the end of a pair of legs
                // despite the fact that this point is still within the detection radius
                // NOTE: this case is rare, typically the current point will be outside the detection radius
                // i.e. will go from sensing the last point of a pair of legs to some point in the distance
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
                // add the current point to the appropriate leg vector
				legs[legIndex].push_back(currPoint);
				// this means that what fails as being the end of a pair of legs within detection radius
				// is considered to be the start of a pair of legs 
            // if on leg2, previous range was a point on a leg, current range is further away
            // and the distance between prev and curr point is large
            // then prev point was the end of a pair of legs
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
			//set previous valid range/point to current range/point
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
    // OUTPUT FOR PYTHON SIMULATION
	//cout << "Legs centre at |" << legsCentre.x << "," << legsCentre.y << endl;
	//cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n";

	debug = false; //full debugging output only ever runs for one laser scan
	publishLegsCentre();
}


/*
 * Checks the current two leg vectors to make sure they satisfy some additional properties about legs
 * If they do, sets the legs centre to the midpoint between the end of leg1 and start of leg2
 * And returns true
 * Otherwise returns false
*/
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
    // Final checks for leg vectors to ensure a valud pair of legs is detected
	if (leg1size>=MIN_LEG_SIZE && leg2size>=MIN_LEG_SIZE && abs(leg1size-leg2size)<=LEG_SIZE_DELTA && p1.distanceTo(p2)<=MAX_LEG_DISTANCE) {
        // valid so set the point the robot will go to
		legsCentre = crosbot::Point2D((p1.x+p2.x)/2.0,(p1.y+p2.y)/2.0);
		return true;
	}
	return false;
}

/*
 * Debugging helper function
 * To print a vector of points
*/
void PolarLegDetector::printPoints(vector<crosbot::Point2D> points) {
	cout <<"[";
	for (int i=0; i<points.size(); i++) {
		crosbot::Point2D p = points[i];
		cout << "("<<p.x<<","<<p.y<<") ";
	}
	cout<<"]\n";
}

/* 
 * Publish the legs centre as a ROS geometry_msgs::Pose
 * Utilising the crosbot poses library to do the conversion
*/
void PolarLegDetector::publishLegsCentre() {
	posePub.publish(crosbot::Pose3D(legsCentre.x,legsCentre.y,0).toROS());
}
