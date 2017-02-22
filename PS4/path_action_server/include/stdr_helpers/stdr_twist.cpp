#include "stdr_twist.h"

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
	
	MOVE_SPEED_ = 1.0;
	TURN_SPEED_ = 0.5;
	TRANS_TIME_ = 0.1;
}

double TwistCommander::performTwist(double duration) {
	while (duration > 0) {
		duration = performTwistIter(duration);
	}
	
	return duration;
}

double TwistCommander::performTwistIter(double duration) {
	if (duration < 0) {
		return duration;
	}
	
	twist_commander_->publish(*twist_cmd_);
	loop_timer_->sleep();
	
	return duration - dt_;
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

double TwistCommander::performStationaryIter(double duration) {
	twist_cmd_->linear.x = 0.0;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	return performTwistIter(duration);
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

double TwistCommander::performForwardIter(double distance) {
	twist_cmd_->linear.x = MOVE_SPEED_;
	twist_cmd_->linear.y = 0.0;
	twist_cmd_->linear.z = 0.0;
	twist_cmd_->angular.x = 0.0;
	twist_cmd_->angular.y = 0.0;
	twist_cmd_->angular.z = 0.0;
	
	return performTwistIter(distance / MOVE_SPEED_) * MOVE_SPEED_;
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
	twist_cmd_->angular.z = TURN_SPEED_ * direction;
	
	return performTwistIter(radians / TURN_SPEED_) * TURN_SPEED_;
}

void TwistCommander::configureTwistParameters(double move_speed, double turn_speed, double trans_time) {
	MOVE_SPEED_ = move_speed;
	TURN_SPEED_ = turn_speed;
	TRANS_TIME_ = trans_time;
}

double TwistCommander::cmdStationary(double duration) {
	return performStationary(duration);
}

double TwistCommander::cmdStationaryIter(double duration) {
	return performStationaryIter(duration);
}

double TwistCommander::cmdForward(double distance) {
	double remaining = performForward(distance);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

double TwistCommander::cmdForwardIter(double distance) {
	return performForwardIter(distance);
}

//template <class T> double TwistCommander::cmdForward(double distance, actionlib::SimpleActionServer<T>* sas) {
//	double remaining = performForward(distance, sas);
//	performStationary(TRANS_TIME_);
//	
//	return remaining;
//}

double TwistCommander::cmdTurn(double radians) {
	int direction = 1;
	if (radians < 0) {
		direction = -1;
		radians *= -1;
	}
	
	double remaining = performTurn(direction, radians);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

double TwistCommander::cmdTurnIter(double radians) {
	int direction = 1;
	if (radians < 0) {
		direction = -1;
		radians *= -1;
	}
	
	return performTurnIter(direction, radians);
}

double TwistCommander::cmdTurn(int direction, double radians) {
	double remaining = performTurn(direction, radians);
	performStationary(TRANS_TIME_);
	
	return remaining;
}

double TwistCommander::cmdTurnIter(int direction, double radians) {
	return performTurnIter(direction, radians);
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

