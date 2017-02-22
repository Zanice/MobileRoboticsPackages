#include "stdr_twist.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>

TwistCommander::TwistCommander(ros::Publisher* twist_commander, ros::Rate* loop_timer, double dt) {
	twist_commander_ = twist_commander;
	loop_timer_ = loop_timer;
	dt_ = dt;
	
	geometry_msgs::Twist twist;
	twist_cmd_ = &twist;
	
	twist_cmd_->linear.x = 0.0;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	MOVE_SPEED_ = 1.0;
	TURN_SPEED_ = 0.5;
	TRANS_TIME_ = 0.1;
}

double TwistCommander::performTwist(double duration) {
	double timer = 0.0;
	
	while (timer < duration) {
		twist_commander_->publish(*twist_cmd_);
		timer += dt_;
		loop_timer_->sleep();
	}
	
	return 0.0;
}


template <class T> double TwistCommander::performTwist(double duration, actionlib::SimpleActionServer<T>* sas) {
	double timer = 0.0;
	
	while (timer < duration) {
		if (sas->isPreemptRequested()) {
			twist_cmd_->linear.x = 0.0;
			twist_cmd_->linear.y = 0.0;
			twist_cmd_->linear.z = 0.0;
			twist_cmd_->angular.x = 0.0;
			twist_cmd_->angular.y = 0.0;
			twist_cmd_->angular.z = 0.0;
			
			while (timer < TRANS_TIME_) {
				twist_commander_->publish(*twist_cmd_);
				timer += dt_;
				loop_timer_->sleep();
			}
			
			return 1.0;
		}
		
		twist_commander_->publish(*twist_cmd_);
		timer += dt_;
		loop_timer_->sleep();
	}
	
	return 0.0;
}

double TwistCommander::performStationary(double duration) {
	twist_cmd_->linear.x = 0.0;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	return performTwist(duration);
}

template <class T> double TwistCommander::performStationary(double duration, actionlib::SimpleActionServer<T>* sas) {
	twist_cmd_->linear.x = 0.0;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	return performTwist(duration, sas);
}

double TwistCommander::performForward(double distance) {
	twist_cmd_->linear.x = MOVE_SPEED_;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	return performTwist(distance / MOVE_SPEED_);
}

template <class T> double TwistCommander::performForward(double distance, actionlib::SimpleActionServer<T>* sas) {
	twist_cmd_->linear.x = MOVE_SPEED_;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	return performTwist(distance / MOVE_SPEED_, sas);
}

double TwistCommander::performTurn(int direction, double radians) {
	// ERROR RESOLVING: make negative radians positive
	if (radians < 0) {
		radians *= -1;
	}
	
	// ERROR RESOLVING: snap direction to ceiling/floor
	if (direction > 1) {
		direction = 1;
	}
	else if (direction < -1) {
		direction = -1;
	}
	
	twist_cmd_->linear.x = 0.0;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = TURN_SPEED_ * direction;
	
	return performTwist(radians / TURN_SPEED_);
}

template <class T> double TwistCommander::performTurn(int direction, double radians, actionlib::SimpleActionServer<T>* sas) {
	// ERROR RESOLVING: make negative radians positive
	if (radians < 0) {
		radians *= -1;
	}
	
	// ERROR RESOLVING: snap direction to ceiling/floor
	if (direction > 1) {
		direction = 1;
	}
	else if (direction < -1) {
		direction = -1;
	}
	
	twist_cmd_->linear.x = 0.0;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = TURN_SPEED_ * direction;
	
	return performTwist(radians / TURN_SPEED_, sas);
}

double TwistCommander::performRightAngleTurn(int direction) {
	return performTurn(direction, M_PI / 2);
}

template <class T> double TwistCommander::performRightAngleTurn(int direction, actionlib::SimpleActionServer<T>* sas) {
	return performTurn(direction, M_PI / 2, sas);
}

void TwistCommander::configureTwistParameters(double move_speed, double turn_speed, double trans_time) {
	MOVE_SPEED_ = move_speed;
	TURN_SPEED_ = turn_speed;
	TRANS_TIME_ = trans_time;
}

double TwistCommander::cmdStationary(double duration) {
	double remaining = performStationary(duration);
	
	return remaining;
}

template <class T> double TwistCommander::cmdStationary(double duration, actionlib::SimpleActionServer<T>* sas) {
	double remaining = performStationary(duration, sas);
	
	return remaining;
}

double TwistCommander::cmdForward(double distance) {
	double remaining = performForward(distance);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

template <class T> double TwistCommander::cmdForward(double distance, actionlib::SimpleActionServer<T>* sas) {
	double remaining = performForward(distance, sas);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

double TwistCommander::cmdTurn(int direction, double radians) {
	double remaining = performTurn(direction, radians);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

template <class T> double TwistCommander::cmdTurn(int direction, double radians, actionlib::SimpleActionServer<T>* sas) {
	double remaining = performTurn(direction, radians, sas);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

double TwistCommander::cmdRightAngleTurn(int direction) {
	double remaining = performRightAngleTurn(direction);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

template <class T> double TwistCommander::cmdRightAngleTurn(int direction, actionlib::SimpleActionServer<T>* sas) {
	double remaining = performRightAngleTurn(direction, sas);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

// -------------------------------------------------

double quaternionToPlanar(geometry_msgs::Quaternion quaternion) {
	double z = quaternion.z;
	double w = quaternion.w;
	double phi = 2 * atan2(z, w);
	
	return phi;
}

double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x2 - x1), 2.0) + pow((y2 - y1), 2.0));
}

double getDeltaPhi(double phi, double reference) {
	return phi - reference;
}

