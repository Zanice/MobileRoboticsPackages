// path_client
// @ Zanice (Zach Janice, znj)
// adapted from "path_client.cpp"

#include <ros/ros.h>
#include <path_trace/PathServiceMessage.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

const double PATH_POINTS[] = {
	0.0, 1.0,
	1.0, 1.0,
	2.0, 2.0
};
const int PATH_POINTS_SIZE = sizeof(PATH_POINTS) / sizeof(PATH_POINTS[0]);

geometry_msgs::Quaternion planarToQuaternion(double phi) {
	geometry_msgs::Quaternion q;
	q.x = 0.0;
	q.y = 0.0;
	q.z = sin(phi / 2);
	q.w = cos(phi / 2);
	
	return q;
}

geometry_msgs::Quaternion getIdentityQuaternion() {
	return planarToQuaternion(0.0);
}

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

geometry_msgs::PoseStamped createNextPose(geometry_msgs::PoseStamped* old_pose, double x, double y) {
	double old_x = (*old_pose).pose.position.x;
	double old_y = (*old_pose).pose.position.y;
	double old_z = (*old_pose).pose.position.z;
	
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

void addPoseToPath(double x, double y, geometry_msgs::PoseStamped* pose, path_trace::PathServiceMessage* path, bool consider_previous) {
	if (consider_previous) {
		*pose = createNextPose(pose, x, y);
	}
	else {
		*pose = createPose(x, y);
	}
	
	(*path).request.nav_path.poses.push_back(*pose);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_trace_client");
	ros::NodeHandle n;
	ros::ServiceClient service = n.serviceClient<path_trace::PathServiceMessage>("path_trace_service");
	
	geometry_msgs::Quaternion quaternion;
	
	bool wait_message_shown = false;
	while (!service.exists()) {
		if (!wait_message_shown) {
			ROS_INFO("Waiting for connection to service...");
		}
		ros::Duration(1.0).sleep();
	}
	
	ROS_INFO("Connection to service established.");
	
	// declare request variables
	geometry_msgs::PoseStamped pose;
	path_trace::PathServiceMessage request;
	
	// start with initial pose
	addPoseToPath(0.0, 0.0, &pose, &request, false);
	
	// build the path from desired coordinates
	int pair_index;
	double x;
	double y;
	for (pair_index = 0; pair_index < PATH_POINTS_SIZE - 1; pair_index += 2) {
		x = PATH_POINTS[pair_index];
		y = PATH_POINTS[pair_index + 1];
		ROS_INFO("Creating pose at point %f, %f.", x, y);
		addPoseToPath(x, y, &pose, &request, true);
	}
	
	// send the request to the service
	service.call(request);
	
	return 0;
}
