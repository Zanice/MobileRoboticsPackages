#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

char* publisher_name = "my_stdr_commander";
char* publish_topic = "/robot0/cmd_vel";

double dt = 0.01;			// update period of 10ms
double speed = 1.0;			// 1m/s speed
double yaw_rate = 0.5;		// 0.5rad/s yaw rate

void be_stationary(double duration, ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	// set command parameters
	(*twist_cmd).linear.x = 0.0;
	(*twist_cmd).linear.y = 0.0;
	(*twist_cmd).linear.z = 0.0;
	(*twist_cmd).angular.x = 0.0;
	(*twist_cmd).angular.y = 0.0;
	(*twist_cmd).angular.z = 0.0;
	
	// perform action
	double timer = 0.0;
	while (timer < duration) {
		(*twist_commander).publish(*twist_cmd);
		timer += dt;
		(*loop_timer).sleep();
	}
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
	double timer = 0.0;
	while (timer < duration) {
		(*twist_commander).publish(*twist_cmd);
		timer += dt;
		(*loop_timer).sleep();
	}
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
	double timer = 0.0;
	while (timer < duration) {
		(*twist_commander).publish(*twist_cmd);
		timer += dt;
		(*loop_timer).sleep();
	}
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
	be_stationary(0.5, &twist_commander, &twist_cmd, &loop_timer);
	
	// -----------------------------
	// NAVIAGE TO THE TO LEFT CORNER
	// -----------------------------
	
	// move forward for 3s
	move_forward(3.0, &twist_commander, &twist_cmd, &loop_timer);
	
	// be stationary for .5s
	be_stationary(0.5, &twist_commander, &twist_cmd, &loop_timer);
	
	make_right_angle_turn(1, &twist_commander, &twist_cmd, &loop_timer);
	
	double timer = 0.0;
	
//	twist_cmd.angular.z=yaw_rate; //and start spinning in place
//	timer=0.0; //reset the timer
//	while(timer<3.0) {
//		twist_commander.publish(twist_cmd);
//		timer+=dt;
//		loop_timer.sleep();
//	}

	twist_cmd.angular.z=0.0; //and stop spinning in place 
	twist_cmd.linear.x=speed; //and move forward again
	timer=0.0; //reset the timer
	while(timer<3.0) {
		twist_commander.publish(twist_cmd);
		timer+=dt;
		loop_timer.sleep();
	}
	//halt the motion
	twist_cmd.angular.z=0.0; 
	twist_cmd.linear.x=0.0; 
	for (int i=0;i<10;i++) {
		twist_commander.publish(twist_cmd);
		loop_timer.sleep();
	}			   
	//done commanding the robot; node runs to completion
}

