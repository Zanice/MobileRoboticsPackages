// path_service
// @ Zanice (Zach Janice, znj)
// adapted from "path_service2.cpp"

#include <ros/ros.h>
#include <path_trace/PathServiceMessage.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

// value in m/s
const double MOVE_SPEED = 1.0;

// value in rads/s
const double TURN_SPEED = 0.5;

// update rate
const double DT = 0.01;

// execute a twist message publish over a certain duration
void perform_twist_action(double duration, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	double timer = 0.0;
	while (timer < duration) {
		(*twist_commander).publish(*twist_cmd);
		timer += DT;
		(*loop_timer).sleep();
	}
}

// execute a twist command for stationary status, for some duration
void be_stationary(double duration, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	// set command parameters
	(*twist_cmd).linear.x = 0.0;
	(*twist_cmd).linear.y = 0.0;
	(*twist_cmd).linear.z = 0.0;
	(*twist_cmd).angular.x = 0.0;
	(*twist_cmd).angular.y = 0.0;
	(*twist_cmd).angular.z = 0.0;
	
	// perform action
	perform_twist_action(duration, twist_commander, twist_cmd, loop_timer);
}

// execute a twist command for moving forward, for some distance in meters
void move_forward(double distance, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	// set command parameters
	(*twist_cmd).linear.x = MOVE_SPEED;
	(*twist_cmd).linear.y = 0.0;
	(*twist_cmd).linear.z = 0.0;
	(*twist_cmd).angular.x = 0.0;
	(*twist_cmd).angular.y = 0.0;
	(*twist_cmd).angular.z = 0.0;
	
	// perform action
	perform_twist_action(distance / MOVE_SPEED, twist_commander, twist_cmd, loop_timer);
}

// execute a twist command for turning positively or negatively, for some amount of radians
void make_turn(int direction, double radians, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	// error checking: snap direction to ceiling/floor
	if (direction > 1) {
		direction = 1;
	}
	else if (direction < -1) {
		direction = -1;
	}
	
	// set command parameters
	(*twist_cmd).linear.x = 0.0;
	(*twist_cmd).linear.y = 0.0;
	(*twist_cmd).linear.z = 0.0;
	(*twist_cmd).angular.x = 0.0;
	(*twist_cmd).angular.y = 0.0;
	(*twist_cmd).angular.z = TURN_SPEED * direction;
	
	// perform action
	perform_twist_action(radians / TURN_SPEED, twist_commander, twist_cmd, loop_timer);
}

// execute a twist command for turning pi/2 radians positively or negatively
void make_right_angle_turn(int direction, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	make_turn(direction, M_PI / 2, twist_commander, twist_cmd, loop_timer);
}

double quaternionToPlanar(geometry_msgs::Quaternion quaternion) {
	double z = quaternion.z;
	double w = quaternion.w;
	double phi = 2 * atan2(z, w);
	
	return phi;
}

bool pathCallback(path_trace::PathServiceMessageRequest& request, path_trace::PathServiceMessageResponse& response) {
	int pose_count = request.nav_path.poses.size();
	ROS_WARN("Received path request of %d poses.", pose_count);
	
	int index;
	geometry_msgs::Pose pose;
	for (index = 0; index < pose_count; index++) {
		pose = request.nav_path.poses[index].pose;
		
		ROS_INFO("<POSE> x=%f y=%f phi=%f", pose.position.x, pose.position.y, quaternionToPlanar(pose.orientation));
	}
	
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_trace_service");
	ros::NodeHandle n;
	
	ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
	
	ros::ServiceServer service = n.advertiseService("path_trace_service", pathCallback);
	ROS_INFO("Ready to accept client requests.");
	ros::spin();
}
