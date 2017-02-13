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
