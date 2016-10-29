### Copyright Elliott Smith, Marie Hansen and Bronte Kalebic, CSE, UNSW 2016

## Welcome to the assignment 2 package
> * Warning, this package requires that you also have installed the **rsa-crosbot** package located at https://github.com/brakespear/rsa-crosbot
The **rsa-ass2** package implements some of the basic functionalities of the Robocup@Home competition.
In particular the package provides nodes for waypoint mapping and person following.

## Hardware
This package has been used on Turtlebots only. However it should work on any mobile platform equipped with an RGB-D camera and a Laser Range Finder at leg height. The topics would likely be different on a different platform.

## Installation
This instalation process is for **catkin** 
Assuming that your catkin workspace is under **~/catkin_ws**, if not replace **~/catkin_ws** with appropriate location. It also assumes you're running Bash shell, if you're running Zsh, source appropriate **setup.zsh** file.
```
cd ~/catkin_ws/src
git clone https://github.com/brakespear/rsa-ass2.git
mv rsa-ass2 ass2
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Running the beacon code
To run the beacon detection and placement code execute
```
roslaunch ass2 imageConverter.launch
roslaunch ass2 beaconMarkers.launch
```
The imageConverter code runs the beacon detection. It looks for two appropriately coloured blobs in the correct relative positions. If a beacon is detected it calculates the distance between the robot and the beacon using the depth image and publishes all the information about the beacon over a topic
The beaconMarkers code reads this message and uses it along with the robots current position to place the beacon in the map. 
**Note that this package does not contain the exploration and mapping code**
For that code and how to run it see the **rsa-crosbot** package and its README

### Subscribed Topics:
* **/camera/rgb/image_color** - video stream (standard ROS image transport)
* **/camera/depth/image_rect** - Depth image
* **/beaconMessage** - Custom ROS message contains beacon information received by beaconMarkers node, details below

### Published Topics:
* **/beaconMessage** - Custom ROS message sent by beacon detection node to beacon placement node
* **/beacons** - Visualisation_msgs::Marker used for placing beacons in map
The following four topics provided a useful debugging mechanism. They are binary images where colour regions appear white and everything else is black
* **/pink/image/binary** 
* **/green/image/binary**
* **/blue/image/binary**
* **/yellow/image/binary**

### Beacon Message
* int8 id
* int32 row
* int32 col
* float64 depth
* string topColour
* string bottomColour
* int32 minRow
* int32 maxRow

## Running the person following code
To run the person following code execute
```
roslaunch ass2 personFollow.launch
```
This node uses the laser range finder to locate the centre of the pair of legs of the (first) person it sees. It then navigates the robot to that position.

### Subscribed Topics:
* **/scan** - sensor_msgs::LaserScan typically from a hokuyo laser
* **/legsCentre** - geometry_msgs::Pose legs centre from the leg detector used by the person tracker to navigate the robot to that position

### Published Topics:
* **/legsCentre** - geometry_msgs::Pose legs centre sent to the person tracker
* **/cmd_vel_mux/input/navi** - geometry_msgs::Twist drive commands to get the robot to track the person

## Using the python simulation
To use the python simulation code uncomment the marked lines in PolarLegDetector.cpp (These should be the only lines producing text output to the console - as such either comment out or remove the ROS\_INFO line in person\_tracking_node.cpp (or strip this line from the output))
Then launch the node and put the output into a text file e.g.
```
roslaunch ass2 personFollow.launch >> simulation.txt
```
Move the python file into the same directory as this text file (if it isn't already), make sure it is executable and execute
```
./vis_polar.py "simulation.txt"
```
A python turtle window should open and display the black points detected by the laser range finder, as well as the red legs centre determined by the leg detection algorithm.