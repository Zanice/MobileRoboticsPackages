// collision_alarm Subscriber
// @ znj
// Based off of provided script "lidar_alarm.cpp" @ wsn

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <math.h>

// node name
const char* NODE_NAME = "collision_alarm";
// name of LIDAR scan topic to subscribe to
const char* SCAN_TOPIC_NAME = "/robot0/laser_0";
// name of alarm topic to publish to
const char* ALARM_TOPIC_NAME = "/collision_alarm/alarm";
// name of minimum distance topic to publish to
const char* MIN_DIST_TOPIC_NAME = "/collision_alarm/min_dist";
// e-stop service to call
const char* ESTOP_SERIVCE_NAME = "estop_service";
// e-stop reset service to call
const char* ESTOP_RESET_SERVICE_NAME = "clear_estop_service";
// time to wait to attempt connection to services agian
const double SERVICE_RETRY_WAIT = 1.0;

const double MIN_SAFE_DIST = 0.6;
const double CONE_ANGLE_FROM_MID = 0.8;

// initialize flag for callback information setup
bool info_setup = false;

// initialize callback information variables
double angle_min = 0.0;
double angle_max = 0.0;
double angle_delta = 0.0;
double range_min = 0.0;
double range_max = 0.0;
int min_index = -1;
int max_index = -1;

// initialize variables used in callback
int scan_index = 0;
float current_dist = 0.0;
float min_dist = 0.0;
bool dist_alarm = false;

// initialize services and publishers
ros::ServiceClient estop_service;
ros::ServiceClient estop_reset_service;
ros::Publisher alarm_publisher;
ros::Publisher dist_publisher;

// service calling resources
std_srvs::Trigger raise_estop;
std_srvs::Trigger clear_estop;

void onLaserCallback(const sensor_msgs::LaserScan& laser_scans) {
	if (!info_setup) {
		angle_min = laser_scans.angle_min;
		angle_max = laser_scans.angle_max;
		angle_delta = laser_scans.angle_increment;
		range_min = laser_scans.range_min;
		range_max = laser_scans.range_max;
		
		min_index = (-CONE_ANGLE_FROM_MID - angle_min) / angle_delta;
		max_index = (-angle_min + CONE_ANGLE_FROM_MID) / angle_delta;
		
		ROS_INFO("Sensing between (from min) angles %f and %f.", -1 - angle_min, -angle_min + 1);
		ROS_INFO("Sensing between indexes %d and %d.", min_index, max_index);
		
		info_setup = true;
	}
	
	// PROCEDURE: find the minimum distance measurement in a range of scan measurements
	min_dist = range_max;
	for (scan_index = min_index; scan_index <= max_index; scan_index++) {
		current_dist = laser_scans.ranges[scan_index];
		if (current_dist < min_dist) {
			min_dist = current_dist;
		}
	}
	
	// if the found minimum is dangerously close, raise the alarm
	if (min_dist < MIN_SAFE_DIST) {
		if (!dist_alarm) {
			ROS_WARN("Obstacle found; alarm sounded!");
			
			// call respective service
			estop_service.call(raise_estop);
		}
		dist_alarm = true;
	}
	else {
		if (dist_alarm) {
			ROS_INFO(">>> Alarm cleared.");
			
			// call respective serivce
			estop_reset_service.call(clear_estop);
		}
		dist_alarm = false;
	}
	
	// broadcast the minimum distance and the alarm status
	std_msgs::Bool alarm_msg;
	alarm_msg.data = dist_alarm;
	alarm_publisher.publish(alarm_msg);
	std_msgs::Float32 dist_msg;
	dist_msg.data = min_dist;
	dist_publisher.publish(dist_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	
	estop_service = n.serviceClient<std_srvs::Trigger>(ESTOP_SERIVCE_NAME);
	estop_reset_service = n.serviceClient<std_srvs::Trigger>(ESTOP_RESET_SERVICE_NAME);
	
	while (!estop_service.exists()) {
		ROS_INFO("Waiting for connection to service...");
		ros::Duration(SERVICE_RETRY_WAIT).sleep();
	}
	
	alarm_publisher = n.advertise<std_msgs::Bool>(ALARM_TOPIC_NAME, 1);
	dist_publisher = n.advertise<std_msgs::Float32>(MIN_DIST_TOPIC_NAME, 1);
	
	ros::Subscriber lidar_subscriber = n.subscribe(SCAN_TOPIC_NAME, 1, onLaserCallback);
	ros::spin();
	
	return 0;
}
