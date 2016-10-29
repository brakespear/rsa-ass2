
#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H


#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <crosbot/handle.hpp>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>

#include <ass2/beacon_msg.h>
#include <ass2/beacon.hpp>

#define PINK_LOW_HUE 150 //160
#define PINK_HIGH_HUE 170 //179
#define PINK_LOW_SAT 0
#define PINK_HIGH_SAT 255
#define PINK_LOW_VAL 0 
#define PINK_HIGH_VAL 255

#define YELLOW_LOW_HUE 20 //22
#define YELLOW_HIGH_HUE 30 //38
#define YELLOW_LOW_SAT 100 //250
#define YELLOW_HIGH_SAT 255
#define YELLOW_LOW_VAL 50 //250
#define YELLOW_HIGH_VAL 255

#define GREEN_LOW_HUE 50 //50//38
#define GREEN_HIGH_HUE 100 //100//75
#define GREEN_LOW_SAT 0 //50
#define GREEN_HIGH_SAT 255 //200
#define GREEN_LOW_VAL 0 //70
#define GREEN_HIGH_VAL 255 //110

#define BLUE_LOW_HUE 100 //122 //100//75
#define BLUE_HIGH_HUE 108 //130//130
#define BLUE_LOW_SAT 0 //125
#define BLUE_HIGH_SAT 255 //150
#define BLUE_LOW_VAL 0  //130
#define BLUE_HIGH_VAL 255 //220

#define MAX_DEPTH  2.5
#define MIN_DEPTH  0.2


class ImageConverter
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Publisher pink_binary_pub;
	image_transport::Publisher yellow_binary_pub;
	image_transport::Publisher green_binary_pub;
	image_transport::Publisher blue_binary_pub;
	image_transport::Subscriber image_sub_;
	ros::Subscriber depth_sub;
	ros::Publisher beacon_pub;	
	Beacon detectedBeacon; //stores all the details of a detected beacon
	std::vector<Beacon> beaconsList; //vector of all the potential beacon combinations to check which beacon was detected

	bool beaconDetected = false;	
	

public:
	//descriptions of these in cpp file
	ImageConverter();
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void depth_callback(const sensor_msgs::ImageConstPtr& depthMsg);
	Beacon getBeaconByColours(std::string top, std::string bottom);
	std::vector<int> getColouredObjectDetectedXY(std::string colour, std_msgs::Header header, cv::Mat hsvImage);
	~ImageConverter();
};


void printCalibrationValuesGrid(cv::Mat hsvImage, std::string hsv, int startRow, int startCol, int width, int height);

#endif

