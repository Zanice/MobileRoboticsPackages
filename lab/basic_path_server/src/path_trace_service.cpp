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

// robot warm-up time
const double WARM_UP_TIME = 0.2;
// robot command transition time
const double TRANS_TIME = 0.1;
// robot movement speed value in m/s
const double MOVE_SPEED = 0.4;
// robot turn speed value in rads/s
const double TURN_SPEED = 0.3;
// time buffer tacked on to rotation commands
const double TURN_WARM_UP_TIME = 1.0;

// update rate
const double DT = 0.01;

// minimum number of command iterations that would make an action worthwhile
const double THRESHOLD_ITERS = 1;
// threshold for movement adjustment, in m
const double MOVE_THRESHOLD = MOVE_SPEED * DT * THRESHOLD_ITERS;
// threshold for turn adjustment, in rads
const double TURN_THRESHOLD = TURN_SPEED * DT * THRESHOLD_ITERS;

// current pose information variables
double current_x = 0.0;
double current_y = 0.0;
double current_phi = 0.0;

//action publishing variables
ros::Publisher* twist_commander;
geometry_msgs::Twist twist_cmd;
ros::Rate* loop_timer;

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
	perform_twist_action((radians / TURN_SPEED) + TURN_WARM_UP_TIME, twist_commander, twist_cmd, loop_timer);
}

// execute a twist command for some amount of positive or negative radians
void make_turn(double radians, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	if (radians > 0) {
		make_turn(1, radians, twist_commander, twist_cmd, loop_timer);
	}
	else {
		make_turn(-1, radians * -1, twist_commander, twist_cmd, loop_timer);
	}
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

double getDistanceBetween(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x2 - x1), 2.0) + pow((y2 - y1), 2.0));
}

double getDeltaPhi(double phi, double reference) {
	return phi - reference;
}

bool pathCallback(path_trace::PathServiceMessageRequest& request, path_trace::PathServiceMessageResponse& response) {
	int pose_count = request.nav_path.poses.size();
	ROS_WARN("Received path request of %d poses.", pose_count);
	
	int index;
	double turn_phi;
	double forward_dist;
	geometry_msgs::Pose pose;
	for (index = 0; index < pose_count; index++) {
		ROS_INFO("Beginning processing of pose %d:", index);
		pose = request.nav_path.poses[index].pose;
		
		// display goal pose, current pose information
		ROS_WARN("\t<POSE %d> x=%f y=%f phi=%f as DESIRED", index, pose.position.x, pose.position.y, quaternionToPlanar(pose.orientation));
		ROS_INFO("\t<POSE %d> x=%f y=%f phi=%f as CURRENT", index, current_x, current_y, current_phi);
		
		// determine the turn to perform as a subgoal for the pose
		turn_phi = getDeltaPhi(quaternionToPlanar(pose.orientation), current_phi);
		
		// check proposed turn against threshold for minimum turn
		if (turn_phi > TURN_THRESHOLD || turn_phi < -TURN_THRESHOLD) {
			// perform and record the expected result of the turn <ERROR CHECKING?>
			ROS_INFO("\t<POSE %d> Turning %f radians.", index, turn_phi);
			make_turn(turn_phi, twist_commander, &twist_cmd, loop_timer);
			be_stationary(TRANS_TIME, twist_commander, &twist_cmd, loop_timer);
			current_phi = current_phi + turn_phi;
		}
		else {
			ROS_WARN("\t<POSE %d> %f RADIANS FALL BELOW THRESHOLD. NO TURNING.", index, turn_phi);
		}
		
		// determine the forward movement distance as a subgoal for the pose
		forward_dist = getDistanceBetween(current_x, current_y, pose.position.x, pose.position.y);
		
		// check proposed movement against threshold for minimum movement
		if (forward_dist > MOVE_THRESHOLD) {
			// perform and record the expected result of the forward movement <ERROR CHECKING?>
			ROS_INFO("\t<POSE %d> Moving forward %f meters.", index, forward_dist);
			move_forward(forward_dist, twist_commander, &twist_cmd, loop_timer);
			be_stationary(TRANS_TIME, twist_commander, &twist_cmd, loop_timer);
			current_x = pose.position.x;
			current_y = pose.position.y;
		}
		else {
			ROS_WARN("\t<POSE %d> %f METERS FALL BELOW THRESHOLD. NO MOVEMENT.", index, forward_dist);
		}
		
		// report the results of attempting the current pose
		ROS_WARN("\t<POSE %d> x=%f y=%f phi=%f as RESULT", index, current_x, current_y, current_phi);
	}
	
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_trace_service");
	ros::NodeHandle n;
	
	// initialize global variables for publishing
	ros::Publisher commander = (n.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
	twist_commander = &commander;
	twist_cmd.linear.x = 0.0;
	twist_cmd.linear.y = 0.0;
	twist_cmd.linear.z = 0.0;
	twist_cmd.angular.x = 0.0;
	twist_cmd.angular.y = 0.0;
	twist_cmd.angular.z = 0.0;
	ros::Rate timer(1 / DT);
	loop_timer = &timer;
	
	// warm up communications with the robot
	be_stationary(WARM_UP_TIME, twist_commander, &twist_cmd, loop_timer);
	
	ros::ServiceServer service = n.advertiseService("path_trace_service", pathCallback);
	ROS_INFO("Ready to accept client requests.");
	ros::spin();
}

