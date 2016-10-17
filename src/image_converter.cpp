
#include <ros/ros.h>
#include "ass2/image_converter.hpp"


using namespace cv;

ImageConverter::ImageConverter()    : it_(nh_) 
{
  ROS_INFO("hello\n");
  // Subscribe to depth and colour images
	depth_sub = nh_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::depth_callback, this);
  image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
	beacon_pub = nh_.advertise<ass2::beacon_msg>("beaconMessage",100);
	//beacon_placed_sub = nh_.subscribe("beaconMessage_return",1, &ImageConverter::beaconCb,this);
	pink_binary_pub = it_.advertise("/pink/image/binary",1);
	yellow_binary_pub = it_.advertise("/yellow/image/binary",1);
	green_binary_pub = it_.advertise("/green/image/binary",1);
	blue_binary_pub = it_.advertise("/blue/image/binary",1);
    ros::NodeHandle paramNh("~");

  // Load beacons
	Beacon genericBeacon;
  beaconsList = genericBeacon.parseBeacons(paramNh);
}

ImageConverter::~ImageConverter()
{
}


// helper function
std::vector<int> ImageConverter::getColouredObjectDetectedXY(std::string colour, std_msgs::Header header, Mat hsvImage) {
	// Find coloured object
	Mat colouredImg;
	int lowHue;
	int highHue;
	int lowSat;
	int highSat;
	int lowVal;
	int highVal;
	if (colour == "pink") {
		lowHue = PINK_LOW_HUE;
		highHue = PINK_HIGH_HUE;
		lowSat = PINK_LOW_SAT;
		highSat = PINK_HIGH_SAT;
		lowVal = PINK_LOW_VAL;
		highVal = PINK_HIGH_VAL;
	} else if (colour == "yellow") {
		lowHue = YELLOW_LOW_HUE;
		highHue = YELLOW_HIGH_HUE;
		lowSat = YELLOW_LOW_SAT;
		highSat = YELLOW_HIGH_SAT;
		lowVal = YELLOW_LOW_VAL;
		highVal = YELLOW_HIGH_VAL;
	} else if (colour == "green") {
		lowHue = GREEN_LOW_HUE;
		highHue = GREEN_HIGH_HUE;
		lowSat = GREEN_LOW_SAT;
		highSat = GREEN_HIGH_SAT;
		lowVal = GREEN_LOW_VAL;
		highVal = GREEN_HIGH_VAL;
	} else if (colour == "blue") {
		lowHue = BLUE_LOW_HUE;
		highHue = BLUE_HIGH_HUE;
		lowSat = BLUE_LOW_SAT;
		highSat = BLUE_HIGH_SAT;
		lowVal = BLUE_LOW_VAL;
		highVal = BLUE_HIGH_VAL;
	} else {
		ROS_ERROR("Invalid colour seach string\n");
	}
		
		
	// Look within given hue, saturation and value range
	inRange(hsvImage, Scalar(lowHue,lowSat,lowVal), Scalar(highHue,highSat,highVal),colouredImg);
	//morphological opening (remove small objects from the foreground)
  	erode(colouredImg, colouredImg, getStructuringElement(MORPH_ELLIPSE, Size(25, 25)) );
  	dilate( colouredImg, colouredImg, getStructuringElement(MORPH_ELLIPSE, Size(25, 25)) ); 

  //morphological closing (fill small holes in the foreground)
	dilate( colouredImg, colouredImg, getStructuringElement(MORPH_ELLIPSE, Size(25, 25)) ); 
	erode(colouredImg, colouredImg, getStructuringElement(MORPH_ELLIPSE, Size(25, 25)) );
	//publish the colour search binary image
	cv_bridge::CvImage binary_msg;
	binary_msg.header   = header; 
	binary_msg.encoding = sensor_msgs::image_encodings::MONO8; 
	binary_msg.image    = colouredImg; 
	if (colour == "pink") {
		pink_binary_pub.publish(binary_msg.toImageMsg());	
	} else if (colour == "yellow") {
		yellow_binary_pub.publish(binary_msg.toImageMsg());	
	} else if (colour == "green") {
		green_binary_pub.publish(binary_msg.toImageMsg());	
	} else if (colour == "blue") {
		blue_binary_pub.publish(binary_msg.toImageMsg());	
	} else {
		ROS_ERROR("Invalid colour search string\n");
	}
	
	Moments objectMoments = moments(colouredImg);
	
	double oM01 = objectMoments.m01;
	double oM10 = objectMoments.m10;
	double oArea = objectMoments.m00;

	int objectDetected = 0;
	int objectX;
	int objectY;
	if (oArea>100000) { //change this value to change the distance at which the beacons are detected bigger->closer, smaller->farther 100000 default
		objectX = oM10/oArea;
		objectY = oM01/oArea;
		objectDetected = 1;
		//ROS_INFO("Detected %s\n", colour.c_str());
	//} else {
	  //ROS_INFO("didn't detect %s\n", colour.c_str());
	}
	std::vector<int> result;
	result.push_back(objectDetected);
	result.push_back(objectX);
	result.push_back(objectY);
	return result;
}


void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	Mat hsvImage;
	cvtColor(cv_ptr->image,hsvImage,COLOR_BGR2HSV);

	// Find pink objects
	std::vector<int> pinkObjectDetails = getColouredObjectDetectedXY("pink",msg->header,hsvImage);
	int pinkObjectDetected = pinkObjectDetails.at(0);
	int pinkObjectX = pinkObjectDetails.at(1);
	int pinkObjectY = pinkObjectDetails.at(2);
	
	// Find yellow objects
	std::vector<int> yellowObjectDetails = getColouredObjectDetectedXY("yellow",msg->header,hsvImage);
	int yellowObjectDetected = yellowObjectDetails.at(0);
	int yellowObjectX = yellowObjectDetails.at(1);
	int yellowObjectY = yellowObjectDetails.at(2);

	// Find green objects
	std::vector<int> greenObjectDetails = getColouredObjectDetectedXY("green",msg->header,hsvImage);
	int greenObjectDetected = greenObjectDetails.at(0);
	int greenObjectX = greenObjectDetails.at(1);
	int greenObjectY = greenObjectDetails.at(2);

	// Find blue objects
	std::vector<int> blueObjectDetails = getColouredObjectDetectedXY("blue",msg->header,hsvImage);
	int blueObjectDetected = blueObjectDetails.at(0);
	int blueObjectX = blueObjectDetails.at(1);
	int blueObjectY = blueObjectDetails.at(2);
	
	/* In the following section the commented out code was used when we were getting the depth of the beacon
	   This code worked but unfortuntely we were unable to use the depth to place the beacon on the map
	*/
	if (/*!yellow_pink_beaconPlaced &&*/ yellowObjectDetected && pinkObjectDetected && yellowObjectY<pinkObjectY) {
		ROS_INFO("Detected yellow top, pink bottom beacon!!!\n");
		yellow_pink_beaconPlaced = true;
		beaconDetected = true;
		detectedBeacon = getBeaconByColours("yellow","pink");
		//ass2::beacon_msg beaconMsg;
		detectedBeacon.beaconMsg.id = detectedBeacon.id;
		detectedBeacon.beaconMsg.topColour = detectedBeacon.top;
		detectedBeacon.beaconMsg.bottomColour = detectedBeacon.bottom;
		detectedBeacon.beaconMsg.col = (yellowObjectX+pinkObjectX)/2;
		detectedBeacon.beaconMsg.row = (yellowObjectY+pinkObjectY)/2;
		detectedBeacon.beaconMsg.minRow = yellowObjectY;
		detectedBeacon.beaconMsg.maxRow = pinkObjectY;
		detectedBeacon.beaconMsg.depth = 0;
		beacon_pub.publish(detectedBeacon.beaconMsg);
		/*
		beaconDetected = true;
		detectedBeacon = getBeaconByColours("yellow","pink");
		detectedBeacon.col = (yellowObjectX+pinkObjectX)/2;
		detectedBeacon.minRow = yellowObjectY;
		detectedBeacon.maxRow = pinkObjectY;
		*/
	} else if (/*!pink_green_beaconPlaced &&*/ greenObjectDetected && pinkObjectDetected && pinkObjectY<greenObjectY) {
		ROS_INFO("Detected pink top, green bottom beacon!!!\n");
		pink_green_beaconPlaced = true;
		beaconDetected = true;
		detectedBeacon = getBeaconByColours("pink","green");
		//ass2::beacon_msg beaconMsg;
		detectedBeacon.beaconMsg.id = detectedBeacon.id;
		detectedBeacon.beaconMsg.topColour = detectedBeacon.top;
		detectedBeacon.beaconMsg.bottomColour = detectedBeacon.bottom;
		detectedBeacon.beaconMsg.col = (greenObjectX+pinkObjectX)/2;
		detectedBeacon.beaconMsg.row = (greenObjectY+pinkObjectY)/2;
		detectedBeacon.beaconMsg.minRow = pinkObjectY;
		detectedBeacon.beaconMsg.maxRow = greenObjectY;
		detectedBeacon.beaconMsg.depth = 0;
		beacon_pub.publish(detectedBeacon.beaconMsg);
		/*
		beaconDetected = true;
		detectedBeacon = getBeaconByColours("pink","green");
		detectedBeacon.col = (greenObjectX+pinkObjectX)/2;
		detectedBeacon.minRow = pinkObjectY;
		detectedBeacon.maxRow = greenObjectY;
		*/
	} else if (/*!blue_pink_beaconPlaced &&*/ blueObjectDetected && pinkObjectDetected && blueObjectY<pinkObjectY) {
		ROS_INFO("Detected blue top, pink bottom beacon!!!\n");
		blue_pink_beaconPlaced = true;
		beaconDetected = true;
		detectedBeacon = getBeaconByColours("blue","pink");
		//ass2::beacon_msg beaconMsg;
		detectedBeacon.beaconMsg.id = detectedBeacon.id;
		detectedBeacon.beaconMsg.topColour = detectedBeacon.top;
		detectedBeacon.beaconMsg.bottomColour = detectedBeacon.bottom;
		detectedBeacon.beaconMsg.col = (blueObjectX+pinkObjectX)/2;
		detectedBeacon.beaconMsg.row = (blueObjectY+pinkObjectY)/2;
		detectedBeacon.beaconMsg.minRow = blueObjectY;
		detectedBeacon.beaconMsg.maxRow = pinkObjectY;
		detectedBeacon.beaconMsg.depth = 0;
		beacon_pub.publish(detectedBeacon.beaconMsg);
		/*
		beaconDetected = true;
		detectedBeacon = getBeaconByColours("blue","pink");
		detectedBeacon.col = (blueObjectX+pinkObjectX)/2;
		detectedBeacon.minRow = blueObjectY;
		detectedBeacon.maxRow = pinkObjectY;
		*/
	} else if (/*!pink_yellow_beaconPlaced &&*/ yellowObjectDetected && pinkObjectDetected && pinkObjectY<yellowObjectY) {
		ROS_INFO("Detected pink top, yellow bottom beacon!!!\n");
		detectedBeacon = getBeaconByColours("pink","yellow");
		pink_yellow_beaconPlaced = true;
		beaconDetected = true;
		//comp3431_starter::beacon_msg beaconMsg;
		detectedBeacon.beaconMsg.id = detectedBeacon.id;
		detectedBeacon.beaconMsg.topColour = detectedBeacon.top;
		detectedBeacon.beaconMsg.bottomColour = detectedBeacon.bottom;
		detectedBeacon.beaconMsg.col = (yellowObjectX+pinkObjectX)/2;
		detectedBeacon.beaconMsg.row = (yellowObjectY+pinkObjectY)/2;
		detectedBeacon.beaconMsg.minRow = pinkObjectY;
		detectedBeacon.beaconMsg.maxRow = yellowObjectY;
		detectedBeacon.beaconMsg.depth = 0;
		beacon_pub.publish(detectedBeacon.beaconMsg);
		/*
		beaconDetected = true;
		detectedBeacon = getBeaconByColours("pink","yellow");
		detectedBeacon.col = (yellowObjectX+pinkObjectX)/2;
		detectedBeacon.minRow = pinkObjectY;
		detectedBeacon.maxRow = yellowObjectY;
		*/	
	}
	
}

Beacon ImageConverter::getBeaconByColours(std::string top, std::string bottom) {
	Beacon thisBeacon;
	for (int i=0; i<beaconsList.size(); i++) {
		thisBeacon = beaconsList[i];
		if (thisBeacon.top == top && thisBeacon.bottom == bottom) {
			return thisBeacon;
		}
	}
	ROS_ERROR("No beacon found with top colour %s and bottom colour %s\n",top.c_str(),bottom.c_str());
}


void printCalibrationValuesGrid(cv::Mat hsvImage, std::string hsv, int startRow, int startCol, int width, int height) {
	int index;
	if (hsv=="hue") {
		index=0;
	} else if (hsv=="sat") {
		index=1;
	} else {
		index=2;
	}
	std::cout << "[";
	for (int r=startRow; r<startRow+height; r++) {
		std::cout << "[";
		for (int c=startCol; c<startCol+width; c++) {
			int val = hsvImage.at<Vec3b>(c,r)[index];
			std::cout << val;
			if (c!=startCol+width-1) {
				std::cout << ",";
			}
		}	
		std::cout << "]";
		if (r!=startRow+height-1) {
			std::cout << std::endl;
		}
	}
	std::cout << "]" << std::endl;
}

/* This callback function and its topic would have been used to continually publish beacon messages
   While centering the beacon in the image before placing it onto the map
	*/
/*
void ImageConverter::beaconCb(const comp3431_starter::beacon_msgPtr& beaconPlaced) {
	if (beaconPlaced->id == 0) {
		yellow_pink_beaconPlaced = true;
	} else if (beaconPlaced->id == 1) {
		pink_green_beaconPlaced = true;
	} else if (beaconPlaced->id == 2) {
		blue_pink_beaconPlaced = true;
	} else if (beaconPlaced->id == 3) {
		pink_yellow_beaconPlaced = true;
	} else {
		ROS_ERROR("Invalid beacon ID returned\n");
	}
}
*/		
		
				


void ImageConverter::depth_callback(const sensor_msgs::ImageConstPtr& depthMsg) {
    if (depthMsg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 && beaconDetected) {
		    ROS_INFO("Depth slice requested for column %d from row %d to %d\n",detectedBeacon.beaconMsg.col,detectedBeacon.beaconMsg.minRow,detectedBeacon.beaconMsg.maxRow);
        int index = (depthMsg->step*detectedBeacon.beaconMsg.minRow)+(4*detectedBeacon.beaconMsg.col);
		    float totalDepth;
		    float numberOfRows = detectedBeacon.beaconMsg.maxRow-detectedBeacon.beaconMsg.minRow;
		    float depth;
		    int i = 0;
		    while (i<=numberOfRows) {
		        //ROS_INFO("in the loop\n");
			      depth = *reinterpret_cast<const float*>(&depthMsg->data[index]);
			      //ROS_INFO("depth is %2f\n", depth);	
			      if (depth>=MIN_DEPTH && depth<=MAX_DEPTH) {
				        totalDepth+=depth;
				        //ROS_INFO("in the if\n");
			      }
			      i++;
			      index+=depthMsg->step; //shift down 1 row
		    }

        float beaconDepth;
        if (numberOfRows) {
  		      beaconDepth = totalDepth/numberOfRows;
        } else {
            beaconDepth = totalDepth;
        }
		    ROS_INFO("Depth calculated as %.2f\n",beaconDepth);
		    ass2::beacon_msg beaconMsg;
		    beaconMsg.id = detectedBeacon.id;
		    beaconMsg.topColour = detectedBeacon.top;
		    beaconMsg.bottomColour = detectedBeacon.bottom;
		    beaconMsg.col = detectedBeacon.beaconMsg.col;
		    beaconMsg.row = (detectedBeacon.beaconMsg.minRow+detectedBeacon.beaconMsg.maxRow)/2;
		    beaconMsg.depth = beaconDepth;
		    beacon_pub.publish(beaconMsg);
		    beaconDetected = false;
	  } else if (!beaconDetected) {
		    ROS_WARN("Depth not requested\n");
    } else {
        ROS_WARN("Invalid image encoding: %s", depthMsg->encoding.c_str());
    }
}



/*int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}*/
