/*
 * Peforms leg detection by creating clusters from the laser scanner data
 * As part of the overall goal of person following
 * Written by Elliott Smith
 * For COMP3431 Assignment 2 Robocup@Home
 * Date: 5/10/2016
*/

#include <ros/ros.h>

#include <ass2/LegDetection.hpp>

#define MIN_RANGE 0.0
#define MAX_RANGE 10.0
#define CLUSTER_DISTANCE 0.025
#define MAX_CLUSTER_SIZE_DELTA 5
#define MAX_LEG_GAP 0.2
#define MAX_CLUSTER_SIZE 40
#define MIN_CLUSTER_SIZE 8
#define GRADIENT_VALUE 0.2 //figure this out


using namespace std;

LegDetector::LegDetector() {
	ros::NodeHandle n;
    scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &LegDetector::callbackScan, this);

}



void LegDetector::callbackScan(const sensor_msgs::LaserScanConstPtr& scan) {
    // Turn laser scan into a point cloud 
	float angle = scan->angle_min;
    for (int n = 0; n < scan->ranges.size(); ++n, angle += scan->angle_increment) {
		if (scan->ranges[n]>MIN_RANGE && scan->ranges[n]<MAX_RANGE) {
			// create 2D crosbot point
			crosbot::Point2D crosbotPoint = crosbot::Point2D(cos(angle) * scan->ranges[n],sin(angle) * scan->ranges[n]);
			// add to point cloud
			pointCloud.push_back(crosbotPoint);
		}
    }
	//find clusters
	findClusters();
	if (debug) {
		//printPointCloud();
		printClusters();
	}
	findLegs();
    if (debug) {
	    cout << "The legs are centered at: " << legsCenter.x << "," << legsCenter.y << endl;
    }
	debug = false;
	
}

void LegDetector::findClusters() {
	clusters.clear();
	crosbot::Point2D prevPoint = pointCloud[0];
	crosbot::Point2D currPoint;
	vector<crosbot::Point2D> currentCluster;
	currentCluster.push_back(prevPoint);
	for (int i=1; i<pointCloud.size(); i++) {
		currPoint = pointCloud[i];
		if (prevPoint.distanceTo(currPoint)>CLUSTER_DISTANCE) {
			clusters.push_back(currentCluster);
			currentCluster.clear();
		}
		currentCluster.push_back(currPoint);
		prevPoint = currPoint;
	}
	clusters.push_back(currentCluster);
}
			

void LegDetector::printClusters() {
	crosbot::Point2D currPoint;
	for (int i=0; i<clusters.size(); i++) {
		vector<crosbot::Point2D> currCluster = clusters[i];
		cout << "[";
		for (int j=0; j<currCluster.size(); j++) {
			currPoint = currCluster[j];
			cout << "(" << currPoint.x << "," << currPoint.y << ") ";
		}
		cout << "]\n";
	}
}

void LegDetector::printPointCloud() {
	crosbot::Point2D currPoint;
	for (int i=0; i<pointCloud.size();i++) {
		currPoint = pointCloud[i];
		cout << "("<<currPoint.x<<","<<currPoint.y<<")\n";
	}
}

void LegDetector::findLegs() {
	vector<crosbot::Point2D> cluster1 = clusters[0];
	vector<crosbot::Point2D> cluster2 = clusters[1];
	vector<crosbot::Point2D> cluster3;
	if (legPair(cluster1,cluster2)) {
		return;
	} else if (singleLegCluster(cluster1)) {
		return;
    }
    /*
	} else if (singleLegCluster(cluster2)) {
		return;
	}
    */
	for (int i=2;i<clusters.size();i++) {
		cluster3 = clusters[i];
		if (legPair(cluster1,cluster3)) {
			break;
		} else if (legPair(cluster2, cluster3)) {
			break;
        }
        /*
		} else if (singleLegCluster(cluster3)) {
			break;
		}
        */
        cluster1 = cluster2;
        cluster2 = cluster3;
	}
}

bool LegDetector::legPair(vector<crosbot::Point2D> cluster1, vector<crosbot::Point2D> cluster2) {
	int cluster1size = cluster1.size();
	int cluster2size = cluster2.size();
	crosbot::Point2D p1;
	crosbot::Point2D p2;
	// First cluster should have size within a certain range
 	// and the two clusters should be similar in size
	if (cluster1size>=MIN_CLUSTER_SIZE && cluster1size<=MAX_CLUSTER_SIZE && cluster2size>=MIN_CLUSTER_SIZE && cluster2size<=MAX_CLUSTER_SIZE && abs(cluster1size-cluster2size)<=MAX_CLUSTER_SIZE_DELTA) {
		p1 = cluster1[cluster1size-1];
		p2 = cluster2[0];
		// the gap  between the legs should be smaller than some value
		if (p1.distanceTo(p2)<=MAX_LEG_GAP) {
			// take the average of the two points
            if (debug) {
                cout << "The two clusters that form the legs are:\n";
            }
            printCluster(cluster1);
            printCluster(cluster2);
			legsCenter = crosbot::Point2D((p1.x+p2.x)/2.0,(p1.y+p2.y)/2.0);
			return true;
		}
	}
	return false;
}

bool LegDetector::singleLegCluster(vector<crosbot::Point2D> cluster) {
	int size = cluster.size();
	// cluster for double leg should be twice the size of single leg
	if (size/2>=MIN_CLUSTER_SIZE && size/2<=MAX_CLUSTER_SIZE) {
		crosbot::Point2D leftPoint = cluster[0];
		crosbot::Point2D middlePoint = cluster[size/2];
		crosbot::Point2D rightPoint = cluster[size-1];
		double m1 = calculateGradient(leftPoint,middlePoint);
		double m2 = calculateGradient(middlePoint,rightPoint);
		//gradients should have opposite signs
		if ((m1<0 &&m2>0) || (m1>0&&m2<0)) {
			// both gradients should have magnitude greater than some value
			if (abs(m1)>GRADIENT_VALUE && abs(m2)>GRADIENT_VALUE) {
				legsCenter = middlePoint;
                cout << "****************************\n";
                cout << "Found single leg cluster\n";
                cout << "****************************\n";
				return true;
			}
		}
	}
	return false;
}


double LegDetector::calculateGradient(crosbot::Point2D p1, crosbot::Point2D p2) {
	return ((p2.y-p1.y)/(p2.x-p1.x));
}

void LegDetector::printCluster(vector<crosbot::Point2D> cluster) {
    if (debug) {
	    cout << "{";
	    for (int i=0; i<cluster.size(); i++) {
		    crosbot::Point2D p = cluster[i];
		    cout << "("<<p.x<<","<<p.y<<") ";
	    }
	    cout << "}\n";
    }
}
			
	

