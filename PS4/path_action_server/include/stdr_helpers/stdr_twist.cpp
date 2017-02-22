#include "stdr_twist.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

TwistCommander::TwistCommander(ros::Publisher* twist_commander, ros::Rate* loop_timer, geometry_msgs::Twist* twist_cmd) {
	return;
}

void TwistCommander::doSomething() {
	ROS_ERROR("I exist!");
}

