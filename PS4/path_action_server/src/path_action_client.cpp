// path_action_client Action Client
// @ znj
// Based off of provided script "example_action_client.cpp" @ wsn

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <path_action_server/pathAction.h>

void onGoalStart() {
	;
}

void onGoalFeedback(const path_action_server::pathFeedbackConstPtr& feedback) {
	;
}

void onGoalCompletion(const actionlib::SimpleClientGoalState& state, const path_action_server::pathResultConstPtr& result) {
	;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_action_client");
	
	path_action_server::pathGoal goal;
	
	actionlib::SimpleActionClient<path_action_server::pathAction> client("path_action", true);
	
	// try to reach server
	
	//client.sendGoal(goal, &onGoalCompletion, &onGoalStart, &onGoalFeedback);
}
