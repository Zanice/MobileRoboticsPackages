#include "twist_helper.h"

#include <ros/ros.h>
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
	
	move_speed_ = 1.0;
	move_speed_factor_ = 1.0;
	move_speed_buffer_ = 0.0;
	turn_speed_ = 0.5;
	turn_speed_factor_ = 1.0;
	turn_speed_factor_ = 0.0;
	trans_time_ = 0.1;
}

double TwistCommander::performTwist(double duration) {
	double remaining = duration;
	double now_remaining;
	
	while (remaining > 0) {
		now_remaining = performTwistIter(remaining);
		remaining = now_remaining;
	}
	
	return remaining;
}

double TwistCommander::performTwistIter(double duration) {
	if (duration < 0) {
		return duration;
	}
	
	twist_commander_->publish(*twist_cmd_);
	loop_timer_->sleep();
	
	double remaining = duration - dt_;
	
	return remaining;
}

double TwistCommander::performStationary(double duration) {
	twist_cmd_->linear.x = 0.0;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	double remaining = performTwist(duration);
	
	return remaining;
}

double TwistCommander::performStationaryIter(double duration) {
	twist_cmd_->linear.x = 0.0;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	double remaining = performTwistIter(duration);
	
	return remaining;
}

double TwistCommander::performForward(double distance) {
	twist_cmd_->linear.x = move_speed_;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	double remaining = performTwist(distance / move_speed_) * move_speed_;
	
	return remaining;
}

double TwistCommander::performForwardIter(double distance) {
	twist_cmd_->linear.x = move_speed_;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	double remaining = performTwistIter(distance / move_speed_) * move_speed_;
	
	return remaining;
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
	twist_cmd_->angular.z = turn_speed_ * direction;
	
	double remaining = performTwist(radians / turn_speed_) * turn_speed_;
	
	return remaining;
}

double TwistCommander::performTurnIter(int direction, double radians) {
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
	twist_cmd_->angular.z = turn_speed_ * direction;
	
	double remaining = performTwistIter(radians / turn_speed_) * turn_speed_;
	
	return remaining;
}

void TwistCommander::configureTwistParameters(double move_speed, double turn_speed, double trans_time) {
	move_speed_ = move_speed;
	turn_speed_ = turn_speed;
	trans_time_ = trans_time;
}

void TwistCommander::configureTwistFactors(double move_speed_factor, double turn_speed_factor) {
	move_speed_factor_ = move_speed_factor;
	turn_speed_factor_ = turn_speed_factor;
}

void TwistCommander::configureTwistBuffers(double move_speed_buffer, double turn_speed_buffer) {
	move_speed_buffer_ = move_speed_buffer;
	turn_speed_buffer_ = turn_speed_buffer;
}

double TwistCommander::cmdStationary(double duration) {
	double remaining = performStationary(duration);
	
	return remaining;
}

double TwistCommander::cmdStationaryIter(double duration) {
	double remaining = performStationaryIter(duration);
	
	return remaining;
}

double TwistCommander::cmdForward(double distance) {
	double remaining = performForward(distance);
	performStationary(trans_time_);
	
	return remaining;
}

double TwistCommander::cmdForwardIter(double distance) {
	double remaining = performForwardIter(distance);
	
	return remaining;
}

double TwistCommander::cmdTurn(int direction, double radians) {
	double remaining = performTurn(direction, radians);
	performStationary(trans_time_);
	
	return remaining;
}

double TwistCommander::cmdTurnIter(int direction, double radians) {
	double remaining = performTurnIter(direction, radians);
	
	return remaining;
}

double rightAnglePhi() {
	return M_PI / 2;
}

geometry_msgs::Quaternion getIdentityQuaternion() {
	return planarToQuaternion(0.0);
}

geometry_msgs::Quaternion planarToQuaternion(double phi) {
	geometry_msgs::Quaternion q;
	q.x = 0.0;
	q.y = 0.0;
	q.z = sin(phi / 2);
	q.w = cos(phi / 2);
	
	return q;
}

double quaternionToPlanar(geometry_msgs::Quaternion quaternion) {
	double z = quaternion.z;
	double w = quaternion.w;
	double phi = 2 * atan2(z, w);
	
	return phi;
}

double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x2 - x1), 2.0) + pow((y2 - y1), 2.0));
}

double shiftPointInDirection(double* x, double* y, double distance, double phi) {
	shiftPointInDirection(x, y, distance, phi, 1);
}

double shiftPointInDirection(double* x, double* y, double distance, double phi, int direction) {
	if (direction > 1) {
		direction = 1;
	}
	else if (direction < -1) {
		direction = -1;
	}
	
	(*x) = (*x) + (direction * cos(phi) * distance);
	(*y) = (*y) + (direction * sin(phi) * distance);
}

double getDeltaPhi(double phi, double reference) {
	double result = phi - reference;
	
	return clampPhi(result);
}

double clampPhi(double phi) {
	while (phi > M_PI * 2) {
		phi -= M_PI * 2;
	}
	while (phi < -(M_PI * 2)) {
		phi += M_PI * 2;
	}
	
	return phi;
}

