// path_action_server Action Server
// @ znj
// Based off of provided script "example_action_server.cpp" @ wsn

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <path_action_server/pathAction.h>

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
		
		void executeGoal(const actionlib::SimpleActionServer<path_action_server::pathAction>::GoalConstPtr& goal);
};

PathActionServer::PathActionServer() : sas_(n_, "path_action", boost::bind(&PathActionServer::executeGoal, this, _1), false) {
	sas_.start();
}

void PathActionServer::executeGoal(const actionlib::SimpleActionServer<path_action_server::pathAction>::GoalConstPtr& goal) {
	;
	
	// sas_.isPreemptRequested()
	// if goal has been cancelled
	// sas_.setAborted(result_);
	
	// sas_.publishFeedback(feedback_);
	
	// sas_.setSucceeded(result_);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_action_server");
	
	PathActionServer server;
	
	ros::spin();
}

