// path_service
// @ Zanice (Zach Janice, znj)
// adapted from "path_service2.cpp"

#include <ros/ros.h>
#include <iostream>
#include <sting>
#include <path_tracing/PathServiceMessage.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

double quaternionToPlanar(geometry_msgs::Quaternion quaternion) {
	double z = quaternion.z;
	double w = quaternion.w;
	double phi = 2 * atan2(z, w);
	
	return phi;
}

bool pathCallback(path_tracing::PathServiceMessageRequest& request, path_tracing::PathServiceMessageResponse response) {
	int pose_count = request.nav_path.poses.size();
	ROS_INFO("Received path request of %d poses.", pose_count);
	
	int index;
	geometry_msgs::Pose pose;
	for (index = 0; index < pose_count; index++) {
		pose = request.nav_path.poses[index].pose;
		
		ROS_INFO("<POSE> x=%f y=%f phi=%f", pose.position.x, pose.position.y, quaternionToPlanar(pose.quaternion));
	}
	
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_service");
	ros::NodeHandle n;
	
	ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
	
	ros::ServiceServer service = n.advertiseService("path_service", pathCallback);
	ROS_INFO("Ready to accept client requests.");
	ros::spin();
}
