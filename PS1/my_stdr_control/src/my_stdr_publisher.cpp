#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

char* publisher_name = "my_stdr_commander";
char* publish_topic = "/robot0/cmd_vel";

double dt = 0.01;			// update period of 10ms
double speed = 1.0;			// 1m/s speed
double yaw_rate = 0.5;		// 0.5rad/s yaw rate

void perform_twist_action(double duration, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	double timer = 0.0;
	while (timer < duration) {
		(*twist_commander).publish(*twist_cmd);
		timer += dt;
		(*loop_timer).sleep();
	}
}

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

void move_forward(double duration, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	// set command parameters
	(*twist_cmd).linear.x = speed;
	(*twist_cmd).linear.y = 0.0;
	(*twist_cmd).linear.z = 0.0;
	(*twist_cmd).angular.x = 0.0;
	(*twist_cmd).angular.y = 0.0;
	(*twist_cmd).angular.z = 0.0;
	
	// perform action
	perform_twist_action(duration, twist_commander, twist_cmd, loop_timer);
}

void make_turn(int direction, double duration, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
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
	(*twist_cmd).angular.z = yaw_rate * direction;
	
	// perform action
	perform_twist_action(duration, twist_commander, twist_cmd, loop_timer);
}

void make_right_angle_turn(int direction, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	make_turn(direction, M_PI, twist_commander, twist_cmd, loop_timer);
}

int main(int argc, char **argv) {
	// initialize the ROS publisher
	ros::init(argc, argv, publisher_name);
	ros::NodeHandle node;
	ros::Publisher twist_commander = node.advertise<geometry_msgs::Twist>(publish_topic, 1);
	
	// configure rate of update for the node
	ros::Rate loop_timer(1 / dt);
	
	// create twist command, initialize as stationary
	geometry_msgs::Twist twist_cmd;
	twist_cmd.linear.x=0.0;
	twist_cmd.linear.y=0.0;
	twist_cmd.linear.z=0.0;
	twist_cmd.angular.x=0.0;
	twist_cmd.angular.y=0.0;
	twist_cmd.angular.z=0.0;
	
	// warm up communication with stationary command
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	// -----------------------------
	// NAVIAGE TO THE TO LEFT CORNER
	// -----------------------------
	
	move_forward(3.5, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	make_right_angle_turn(1, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	move_forward(3.0, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	make_right_angle_turn(-1, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	move_forward(3.5, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	make_right_angle_turn(1, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	move_forward(2.25, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	make_turn(1, 3.0, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	move_forward(6.0, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	make_right_angle_turn(-1, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	move_forward(2.0, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	make_right_angle_turn(1, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	move_forward(0.75, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	make_right_angle_turn(-1, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
	
	move_forward(4.5, &twist_commander, &twist_cmd, &loop_timer);
	be_stationary(0.1, &twist_commander, &twist_cmd, &loop_timer);
}

