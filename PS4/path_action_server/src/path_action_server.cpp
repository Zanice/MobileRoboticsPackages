// path_action_server Action Server
// @ znj
// Based off of provided script "example_action_server.cpp" @ wsn

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <path_action_server/pathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include "stdr_helpers/stdr_twist.h"

// - - - - - - - - - - - - - - -
// CONSTANT AND GLOBAL VARIABLES
// - - - - - - - - - - - - - - -

// node name
const char* NODE_NAME = "path_action_server";
// service name
const char* SERVICE_NAME = "path_action_service";
// name of velocity topic to publish to
const char* VEL_TOPIC_NAME = "/robot0/cmd_vel";

// robot warm-up time
const double WARM_UP_TIME = 0.2;
// robot command transition time
const double TRANS_TIME = 0.1;
// robot movement speed value in m/s
const double MOVE_SPEED = 1.0;
// robot turn speed value in rads/s
const double TURN_SPEED = 0.5;

// update rate
const double DT = 0.01;

// minimum number of command iterations that would make an action worthwhile
const double THRESHOLD_ITERS = 1;
// threshold for movement adjustment, in m
const double MOVE_THRESHOLD = MOVE_SPEED * DT * THRESHOLD_ITERS;
// threshold for turn adjustment, in rads
const double TURN_THRESHOLD = TURN_SPEED * DT * THRESHOLD_ITERS;

// current pose information variables
double current_x_ = 0.0;
double current_y_ = 0.0;
double current_phi_ = 0.0;

// action publishing variables
ros::Publisher* twist_commander_;
ros::Rate* loop_timer_;

// - - - - - - - - - -
// HELPER METHOD STUBS
// - - - - - - - - - -

bool perform_twist_action(double duration, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer, actionlib::SimpleActionServer<path_action_server::pathAction>* sas);
bool be_stationary(double duration, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer, actionlib::SimpleActionServer<path_action_server::pathAction>* sas);
bool move_forward(double distance, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer, actionlib::SimpleActionServer<path_action_server::pathAction>* sas);
bool make_turn(int direction, double radians, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer, actionlib::SimpleActionServer<path_action_server::pathAction>* sas);
bool make_turn(double radians, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer, actionlib::SimpleActionServer<path_action_server::pathAction>* sas);
bool make_right_angle_turn(int direction, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer, actionlib::SimpleActionServer<path_action_server::pathAction>* sas);
double quaternionToPlanar(geometry_msgs::Quaternion quaternion);
double getDistanceBetween(double x1, double y1, double x2, double y2);
double getDeltaPhi(double phi, double reference);

// - - - - - - - - - -
// CLASSES AND METHODS
// - - - - - - - - - -

// class encompassing server object, functionality
class PathActionServer {
	private:
		ros::NodeHandle n_;
		actionlib::SimpleActionServer<path_action_server::pathAction> sas_;
		
		path_action_server::pathGoal goal_;
		path_action_server::pathResult result_;
		path_action_server::pathFeedback feedback_;
		
	public:
		PathActionServer();
		
		~PathActionServer(void) {
			;
		}
		
		void onFailedGoal(path_action_server::pathGoal* goal, path_action_server::pathResult* result, actionlib::SimpleActionServer<path_action_server::pathAction>* sas, int last_full_pose);
		void executeGoal(const actionlib::SimpleActionServer<path_action_server::pathAction>::GoalConstPtr& goal);
};

// server class constructor
PathActionServer::PathActionServer() : sas_(n_, SERVICE_NAME, boost::bind(&PathActionServer::executeGoal, this, _1), false) {
	// start the service
	sas_.start();
}

// performs a reporting procedure via the result on the status of a goal that failed
void PathActionServer::onFailedGoal(path_action_server::pathGoal* goal, path_action_server::pathResult* result, actionlib::SimpleActionServer<path_action_server::pathAction>* sas, int last_full_pose) {
	ROS_ERROR("Current path was aborted by the client.");
	
	// add unfinished poses to the result report
	int index;
	int pose_count = goal->nav_path.poses.size();
	geometry_msgs::PoseStamped current_pose;
	for (index = last_full_pose + 1; index < pose_count; index++) {
		current_pose = goal->nav_path.poses[index];
		result->nav_path.poses.push_back(current_pose);
	}
	
	// report the pose the robot ended on
	geometry_msgs::Pose end_pose;
	result->end_pose = end_pose;
	
	sas->setAborted(*result);
}

// receives and executes a goal from the client
void PathActionServer::executeGoal(const actionlib::SimpleActionServer<path_action_server::pathAction>::GoalConstPtr& goal) {
	// create a twist commander instance
	TwistCommander commander(twist_commander_, loop_timer_, DT);
	
	int pose_count = goal->nav_path.poses.size();
	ROS_WARN("Received path request of %d poses.", pose_count);
	
	// set up the instance's variables
	goal_ = *goal;
	feedback_.last_full_pose = -1;
	path_action_server::pathResult result;
	result_ = result;
	
	int index;
	double turn_phi;
	int turn_dir;
	double forward_dist;
	geometry_msgs::Pose pose;
	double twist_left;
	for (index = 0; index < pose_count; index++) {
		ROS_INFO("Beginning processing of pose %d:", index);
		pose = goal->nav_path.poses[index].pose;
		
		// display goal pose, current pose information
		ROS_WARN("\t<POSE %d> x=%f y=%f phi=%f as DESIRED", index, pose.position.x, pose.position.y, quaternionToPlanar(pose.orientation));
		ROS_INFO("\t<POSE %d> x=%f y=%f phi=%f as CURRENT", index, current_x_, current_y_, current_phi_);
		
		// determine the turn to perform as a subgoal for the pose
		turn_phi = getDeltaPhi(quaternionToPlanar(pose.orientation), current_phi_);
		turn_dir = 1;
		if (turn_phi < 0) {
			turn_dir = -1;
			turn_phi *= -1;
		}
		
		// check proposed turn against threshold for minimum turn
		if (turn_phi > TURN_THRESHOLD || turn_phi < -TURN_THRESHOLD) {
			// perform and record the expected result of the turn <ERROR CHECKING?>
			ROS_INFO("\t<POSE %d> Turning %f radians.", index, turn_phi);
			twist_left = turn_phi;
			while (twist_left > 0 && !sas_.isPreemptRequested()) {
				ROS_INFO("INSIDE LOOP");
				twist_left = commander.cmdTurnIter(turn_dir, twist_left);
				ROS_INFO("OUTSIDE LOOP");
			}
			
			// if the turn was interrupted, run the abort procedure
			if (twist_left > 0) {
				onFailedGoal(&goal_, &result_, &sas_, feedback_.last_full_pose);
				return;
			}
			
			current_phi_ = current_phi_ + turn_phi;
		}
		else {
			ROS_WARN("\t<POSE %d> %f RADIANS FALL BELOW THRESHOLD. NO TURNING.", index, turn_phi);
		}
		
		// determine the forward movement distance as a subgoal for the pose
		forward_dist = distanceBetweenPoints(current_x_, current_y_, pose.position.x, pose.position.y);
		
		// check proposed movement against threshold for minimum movement
		if (forward_dist > MOVE_THRESHOLD) {
			// perform and record the expected result of the forward movement <ERROR CHECKING?>
			ROS_INFO("\t<POSE %d> Moving forward %f meters.", index, forward_dist);
			twist_left = forward_dist;
			while (twist_left > 0 && !sas_.isPreemptRequested()) {
				ROS_INFO("INSIDE LOOP");
				//twist_left = commander.cmdForwardIter(twist_left);
				commander.cmdForwardIter(twist_left);
				ROS_INFO("DONE");
				twist_left -= 0.01;
				ROS_INFO("OUTSIDE LOOP");
			}
			
			
			// if the forward movement was interrupted, run the abort procedure
			if (twist_left > 0) {
				onFailedGoal(&goal_, &result_, &sas_, feedback_.last_full_pose);
				return;
			}
			
			current_x_ = pose.position.x;
			current_y_ = pose.position.y;
		}
		else {
			ROS_WARN("\t<POSE %d> %f METERS FALL BELOW THRESHOLD. NO MOVEMENT.", index, forward_dist);
		}
		
		// report the results of attempting the current pose
		ROS_WARN("\t<POSE %d> x=%f y=%f phi=%f as RESULT", index, current_x_, current_y_, current_phi_);
		
		feedback_.last_full_pose = index;
		sas_.publishFeedback(feedback_);
	}
	
	sas_.setSucceeded(result_);
}

// - - - - - -
// MAIN METHOD
// - - - - - -

int main(int argc, char** argv) {
	// initialize ROS
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	
	// initialize global variables for publishing
	ros::Publisher commander = n.advertise<geometry_msgs::Twist>(VEL_TOPIC_NAME, 1);
	twist_commander_ = &commander;
	ros::Rate timer(1 / DT);
	loop_timer_ = &timer;
	
	// create an instance of the action server
	PathActionServer server;
	
	// spin the process
	ros::spin();
}

