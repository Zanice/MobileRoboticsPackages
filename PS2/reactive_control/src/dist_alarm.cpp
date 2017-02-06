// dist_alarm Subscriber
// @ znj
// Based off of provided script "lidar_alarm.cpp" @ wsn

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

const double MIN_SAFE_DIST = 1.0;

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
int index = 0;
float current_dist = 0.0;
float min_dist = 0.0;
bool dist_alarm = false;

// initialize publishers
ros::Publisher alarm_publisher;
ros::Publisher dist_publisher;

void onLaserCallback(const sensor_msgs::LaserScan& laserScan) {
	if (!info_setup) {
		angle_min = laser_scan.angle_min;
		angle_max = laser_scan.angle_max;
		angle_delta = laser_scan.angle_increment;
		range_min = laser_scan.range_min;
		range_max = laser_scan.range_max;
		
		// from some range of indexes, take the results of the scans
		min_index = 0;
		max_index = 0;
		
		info_setup = true;
	}
	
	// PROCEDURE: find the minimum distance measurement in a range of scan measurements
	min_dist = range_max;
	for (index = min_index; index <= max_index; index++) {
		current_dist = laser_scans.ranges[index];
		if (current_dist < min_dist) {
			min_dist = current_dist;
		}
	}
	
	// if the found minimum is dangerously close, raise the alarm
	if (min_dist < MIN_SAFE_DIST) {
		dist_alarm = true;
	}
	else {
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
	ros::init(argc, argv, "dist_alarm");
	ros::NodeHandle n;
	
	alarm_publisher = n.advertise<std_msgs::Bool>("alarm", 1);
	dist_publisher = n.advertise<std_msgs::Float32>("dist", 1);
	
	ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, onLaserCallback);
	ros::spin();
	
	return 0;
}

