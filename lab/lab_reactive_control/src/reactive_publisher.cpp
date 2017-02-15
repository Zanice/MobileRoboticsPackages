// reactive_publisher Publisher
// @ znj
// Based off of provided script "reactive_commander.cpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>

const double dt = 0.01;			// update period of 10ms
const double speed = 0.2;		// 1m/s speed
const double yaw_rate = 0.5;	// 0.5rad/s yaw rate

bool dist_alarm = false;

void onAlarmCallback(const std_msgs::Bool& alarm_msg) {
	dist_alarm = alarm_msg.data;
}

void set_stationary_cmd(geometry_msgs::Twist* twist_cmd) {
	// set command parameters
	(*twist_cmd).linear.x = 0.0;
	(*twist_cmd).linear.y = 0.0;
	(*twist_cmd).linear.z = 0.0;
	(*twist_cmd).angular.x = 0.0;
	(*twist_cmd).angular.y = 0.0;
	(*twist_cmd).angular.z = 0.0;
}

void set_forward_cmd(geometry_msgs::Twist* twist_cmd) {
	// set command parameters
	(*twist_cmd).linear.x = speed;
	(*twist_cmd).linear.y = 0.0;
	(*twist_cmd).linear.z = 0.0;
	(*twist_cmd).angular.x = 0.0;
	(*twist_cmd).angular.y = 0.0;
	(*twist_cmd).angular.z = 0.0;
}

void set_turn_cmd(int direction, geometry_msgs::Twist* twist_cmd) {
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
}

void perform_twist(ros::Publisher* twist_commander, geometry_msgs::Twist* twist_cmd, ros::Rate* loop_timer) {
	(*twist_commander).publish(*twist_cmd);
	ros::spinOnce();
	(*loop_timer).sleep();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "reactive_commander");
	ros::NodeHandle n;
	
	ros::Subscriber alarm_subscriber = n.subscribe("alarm", 1, onAlarmCallback);
	ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::Rate loop_timer(1 / dt);
	
	geometry_msgs::Twist twist_cmd;
	
	// warm up communications
	set_stationary_cmd(&twist_cmd);
	for (int iteration = 0; iteration < 10; iteration++) {
		perform_twist(&twist_commander, &twist_cmd, &loop_timer);
	}
	
	// perform the main behavioral loop
	bool first_iteration;
	while (ros::ok()) {
		// move forward for as long as the alarm is not raised
		first_iteration = true;
		set_forward_cmd(&twist_cmd);
		while (!dist_alarm) {
			if (first_iteration) {
				ROS_INFO("(alarm=f) Moving forward...");
				first_iteration = false;
			}
			
			perform_twist(&twist_commander, &twist_cmd, &loop_timer);
		}
		
		// the alarm is raised, keep turning to find a new route (until alarm clears)
		first_iteration = true;
		if (rand() % 2 == 0) {
			// randomly turn left
			set_turn_cmd(1, &twist_cmd);
		}
		else {
			// randomly turn right
			set_turn_cmd(-1, &twist_cmd);
		}
		while (dist_alarm) {
			if (first_iteration) {
				ROS_WARN("(alarm=t) Obstacle found. Turning...");
				first_iteration = false;
			}
			
			perform_twist(&twist_commander, &twist_cmd, &loop_timer);
		}
	}
}

