#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

class TwistCommander {
	private:
		double move_speed_;
		double move_speed_factor_;
		double move_speed_buffer_;
		double turn_speed_;
		double turn_speed_factor_;
		double turn_speed_buffer_;
		double trans_time_;
		
		ros::Publisher* twist_commander_;
		ros::Rate* loop_timer_;
		double dt_;
		geometry_msgs::Twist* twist_cmd_;
		
		double performTwist(double duration);
		double performTwistIter(double duration);
		double performStationary(double duration);
		double performStationaryIter(double duration);
		double performForward(double distance);
		double performForwardIter(double distance);
		double performTurn(int direction, double radians);
		double performTurnIter(int direction, double radians);
		
	public:
		TwistCommander(ros::Publisher* twist_commander, ros::Rate* loop_timer, double dt);
		
		void configureTwistParameters(double move_speed, double turn_speed, double trans_time);
		void configureTwistFactors(double move_speed_factor, double turn_speed_factor);
		void configureTwistBuffers(double move_speed_buffer, double turn_speed_buffer);
		
		double cmdStationary(double duration);
		double cmdStationaryIter(double duration);
		double cmdForward(double distance);
		double cmdForwardIter(double distance);
		double cmdTurn(int direction, double radians);
		double cmdTurnIter(int direction, double radians);
};

double rightAnglePhi();

geometry_msgs::Quaternion getIdentityQuaternion();
geometry_msgs::Quaternion planarToQuaternion(double phi);
double quaternionToPlanar(geometry_msgs::Quaternion quaternion);
double distanceBetweenPoints(double x1, double y1, double x2, double y2);
double shiftPointInDirection(double* x, double* y, double distance, double phi);
double shiftPointInDirection(double* x, double* y, double distance, double phi, int direction);
double getDeltaPhi(double phi, double reference);
double clampPhi(double phi);

