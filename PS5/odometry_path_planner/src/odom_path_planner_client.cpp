// "odom_path_planner_client" Action Server Client
// @Zanice
// based off of "example_move_base_client.cpp" @wsn

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>

#include "stdr_helpers/stdr_twist.h"

// - - - - - - - - - - - - - - -
// CONSTANT AND GLOBAL VARIABLES
// - - - - - - - - - - - - - - -

// node name
const char* NODE_NAME = "odom_path_planner_client";
// server name
const char* SERVER_NAME = "odom_path_planner";
// frame id name
const char* FRAME_ID = "/map";

// link frame
const char* LINK_FRAME = "map";
// base link frame
const char* BASE_LINK_FRAME = "base_link";
// lookup failure pause
const double LOOKUP_FAILURE_PAUSE = 1.0;
// server connection failure pause
const double CONNECT_FAILURE_PAUSE = 1.0;
// time to expect result during
const double EXPECT_RESULT_DURATION = 120.0;

// destination pose
geometry_msgs::PoseStamped dest_pose_;

// - - - - - - - - - -
// HELPER METHOD STUBS
// - - - - - - - - - -

void setDestinationPose(double x, double y, double phi);
void onNavigatorDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

// - - - - - - - - - -
// CLASSES AND METHODS
// - - - - - - - - - -

// - - - - - -
// MAIN METHOD
// - - - - - -

int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	
	tf::TransformListener tf_listener;
	geometry_msgs::PoseStamped current_pose;
	move_base_msgs::MoveBaseGoal goal;
	XformUtils xform_utils;
	
	bool tf_lookup_error = true;
	tf::StampedTransform tf_transform;
	while (tf_lookup_error) {
		try {
			tf_listener.lookupTransform(LINK_FRAME, BASE_LINK_FRAME, ros::Time(0), tf_transform);
			tf_lookup_error = false;
		} catch(tf::TransformException& exception) {
			ROS_WARN("%s; Retrying lookup...", exception.what());
			tf_lookup_error = true;
			
			ros::Duration(LOOKUP_FAILURE_PAUSE).sleep();
			ros::spinOnce();
		}
	}
	
	current_pose = xform_utils.get_pose_from_stamped_tf(tf_transform);
	double c_p_x = current_pose.pose.position.x;
	double c_p_y = current_pose.pose.position.y;
	double c_o_z = current_pose.pose.orientation.z;
	double c_o_w = current_pose.pose.orientation.w;
	ROS_INFO("Lookup successful! Current pose is p.x=%f p.y=%f o.z=%f o.w=%f", c_p_x, c_p_y, c_o_z, c_o_w);
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> server(SERVICE_NAME, true);
	
	bool server_connection = false;
	while (!server_connection && ros::ok()) {
		server_connection = server.waitForServer(ros::Duration(CONNECT_FAILURE_PAUSE));
		ros::spinOnce();
		ros::Duration(CONNECT_FAILURE_PAUSE).sleep();
	}
	
	goal.target_pose = dest_pose_;
	server.sendGoal(goal, &onNavigatorDone);
	
	bool finished_before_timeout = server.waitForResult(ros::Duration(EXPECT_RESULT_DURATION));
	if (!finished_before_timeout) {
		ROS_ERROR("Goal timed out; giving up on waiting for the result");
		return 1;
	}
	
	return 0;
}

// - - - - - - - -
// HELPER METHODS
// - - - - - - - -

void setDestinationPose(double x, double y, double phi) {
	dest_pose_.header.frame_id = FRAME_ID;
	dest_pose_.header.stamp = ros::Time::now();
	dest_pose_.pose.position.x = x;
	dest_pose_.pose.position.y = y;
	dest_pose_.pose.position.z = 0;
	dest_pose_.pose.orientation = planarToQuaternion(phi);
}

void onNavigatorDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
	ROS_INFO(">> NAVIGATOR IS DONE: Server responded with state: [%s]", state.toString().c_str());
}

