//pub_des_state_path_client:
// illustrates how to send a request to the append_path_queue_service service

#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

const double PATH_POINTS[] = {
	0.0, -32.0,
	-1.5, -32.0,
	0.0, -32.0,
	0.0, 0.0
};
const int PATH_POINTS_SIZE = sizeof(PATH_POINTS) / sizeof(PATH_POINTS[0]);

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;
    
    // build the path from desired coordinates
	int pair_index;
	double current_x;
	double current_y;
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.frame_id = "world";
	geometry_msgs::Pose pose;
	for (pair_index = 0; pair_index < PATH_POINTS_SIZE - 1; pair_index += 2) {
		current_x = PATH_POINTS[pair_index];
		current_y = PATH_POINTS[pair_index + 1];
		quat = convertPlanarPhi2Quaternion(0);
	
		ROS_INFO("Creating pose at point %f, %f.", current_x, current_y);
		pose.position.x = current_x;
		pose.position.y = current_y;
		pose.position.z = 0.0;
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
	}
	
	client.call(path_srv);

	return 0;
}
