// path_action_client Action Client
// @ znj
// Based off of provided script "example_action_client.cpp" @ wsn

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <path_action_server/pathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <math.h>

#include <twist_helper/twist_helper.h>

// - - - - - - - - - - - - - - -
// CONSTANT AND GLOBAL VARIABLES
// - - - - - - - - - - - - - - -

// node name
const char* NODE_NAME = "bpas_client";
// service name
const char* SERVICE_NAME = "bpas_service";
// name of alarm topic to subscribe to
const char* ALARM_TOPIC_NAME = "/bpas_alarm/alarm";

// path points, in (x, y) pairs
const double PATH_POINTS[] = {
	3.0, 0.0,
	7.0, 4.0,
	7.0, 5.25,
	1.9, 5.25,
	0.15, 7.0,
	0.15, 12.0
};
// length of the path points array
const int PATH_POINTS_SIZE = sizeof(PATH_POINTS) / sizeof(PATH_POINTS[0]);
// distance to revert on failed path
const double REVERT_DIST = 1.0;

// client connection to the server
actionlib::SimpleActionClient<path_action_server::pathAction>* client_;
// current status of the collision alarm
bool collision_alarm_ = false;

// - - - - - - - - - -
// HELPER METHOD STUBS
// - - - - - - - - - -

geometry_msgs::PoseStamped createPose(double x, double y);
geometry_msgs::PoseStamped createNextPose(geometry_msgs::PoseStamped* old_pose, double x, double y);
void addPoseToPath(double x, double y, geometry_msgs::PoseStamped* pose, path_action_server::pathGoal* path, bool consider_previous);

// - - - - - - - - - -
// CLASSES AND METHODS
// - - - - - - - - - -

// callback method on when the collision alarm is raised
void onCollisionAlarm(const std_msgs::Bool& alarm_msg) {
	if (alarm_msg.data && !collision_alarm_) {
		ROS_WARN(">> ALARM RAISED! Cancelling current goal.");
		client_->cancelGoal();
	}
	else if (!alarm_msg.data && collision_alarm_) {
		ROS_WARN(">> Alarm cleared.");
	}
	
	collision_alarm_ = alarm_msg.data;
}

// callback method on when the forwarded goal has been started by the server
void onGoalStart() {
	ROS_INFO("Action server began our requested path.");
}

// callback method on when there is feedback from the server on the forwarded goal
void onGoalFeedback(const path_action_server::pathFeedbackConstPtr& feedback) {
	ROS_INFO("Action server successfully completed up to pose %d.", feedback->last_full_pose);
}

// callback method on when the forwarded goal is completed by the server
void onGoalCompletion(const actionlib::SimpleClientGoalState& state, const path_action_server::pathResultConstPtr& result) {
	int pose_count = result->nav_path.poses.size();
	
	if (pose_count == 0) {
		ROS_INFO("Action server finished our requested path.");
		return;
	}
	
	ROS_WARN("Action server did not fulfill request; constructing new goal to send.");
	
	double halted_x = result->end_pose.position.x;
	double halted_y = result->end_pose.position.y;
	double halted_phi = quaternionToPlanar(result->end_pose.orientation);
	
	double reverted_x = halted_x;
	double reverted_y = halted_y;
	shiftPointInDirection(&reverted_x, &reverted_y, REVERT_DIST, halted_phi, -1);
	
	// create new goal object and start with reversion pose
	path_action_server::pathGoal goal;
	geometry_msgs::PoseStamped pose = createPose(halted_x, halted_y);
	addPoseToPath(reverted_x, reverted_y, &pose, &goal, true);
	
	// build the goal from uncompleted poses
	int pose_index;
	double x;
	double y;
	for (pose_index = 0; pose_index < pose_count; pose_index++) {
		x = result->nav_path.poses[pose_index].pose.position.x;
		y = result->nav_path.poses[pose_index].pose.position.y;
		addPoseToPath(x, y, &pose, &goal, true);
	}
	
	ROS_WARN("Sending updated path to action server.");
	
	// send the request to the service
	client_->sendGoal(goal, &onGoalCompletion, &onGoalStart, &onGoalFeedback);
}

// - - - - - -
// MAIN METHOD
// - - - - - -

int main(int argc, char** argv) {
	// initialize ROS
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	
	// initialize subscription to the alarm and the server connection
	ros::Subscriber alarm_subscriber = n.subscribe(ALARM_TOPIC_NAME, 1, onCollisionAlarm);
	actionlib::SimpleActionClient<path_action_server::pathAction> client(SERVICE_NAME, true);
	client_ = &client;
	
	// wait for a verified server to connect to
	bool wait_message_shown = false;
	while (!client.waitForServer(ros::Duration(1.0))) {
		if (!wait_message_shown) {
			ROS_INFO("Waiting for connection to action server...");
			wait_message_shown = true;
		}
	}
	ROS_INFO("Connection to action server established.");
	
	// declare request variables
	geometry_msgs::PoseStamped pose;
	path_action_server::pathGoal goal;
	
	// start with initial pose
	addPoseToPath(0.0, 0.0, &pose, &goal, false);
	
	// build the path from desired coordinates
	int pair_index;
	double x;
	double y;
	for (pair_index = 0; pair_index < PATH_POINTS_SIZE - 1; pair_index += 2) {
		x = PATH_POINTS[pair_index];
		y = PATH_POINTS[pair_index + 1];
		addPoseToPath(x, y, &pose, &goal, true);
	}
	
	// send the request to the service
	client_->sendGoal(goal, &onGoalCompletion, &onGoalStart, &onGoalFeedback);
	
	// spin the process
	ros::spin();
}

// - - - - - - - -
// HELPER METHODS
// - - - - - - - -

// create a pose from the given (x, y) pair
geometry_msgs::PoseStamped createPose(double x, double y) {
	geometry_msgs::Pose p;
	p.position.x = x;
	p.position.y = y;
	p.position.z = 0.0;
	p.orientation = getIdentityQuaternion();
	
	geometry_msgs::PoseStamped ps;
	ps.pose = p;
	
	return ps;
}

// create a pose from the given (x, y) pair, as a "next step" from a given previous pose
geometry_msgs::PoseStamped createNextPose(geometry_msgs::PoseStamped* old_pose, double x, double y) {
	double old_x = old_pose->pose.position.x;
	double old_y = old_pose->pose.position.y;
	double old_z = old_pose->pose.position.z;
	
	double dx = x - old_x;
	double dy = y - old_y;
	double phi = atan2(dy, dx);
	
	geometry_msgs::Pose p;
	p.position.x = x;
	p.position.y = y;
	p.position.z = old_z;
	p.orientation = planarToQuaternion(phi);
	
	geometry_msgs::PoseStamped ps;
	ps.pose = p;
	
	return ps;
}

// create a pose for the (x, y) pair - dependent on if the previous pose should be considered - and add the new pose to the path in construction
void addPoseToPath(double x, double y, geometry_msgs::PoseStamped* pose, path_action_server::pathGoal* path, bool consider_previous) {
	ROS_INFO("Creating pose at point %f, %f.", x, y);
	
	if (consider_previous) {
		*pose = createNextPose(pose, x, y);
	}
	else {
		*pose = createPose(x, y);
	}
	
	path->nav_path.poses.push_back(*pose);
}

