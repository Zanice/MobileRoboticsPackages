#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

class TwistCommander {
	private:
		double move_speed_;
		double turn_speed_;
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
		
		double cmdStationary(double duration);
		double cmdStationaryIter(double duration);
		double cmdForward(double distance);
		double cmdForwardIter(double distance);
		double cmdTurn(double radians);
		double cmdTurnIter(double radians);
		double cmdTurn(int direction, double radians);
		double cmdTurnIter(int direction, double radians);
};

double rightAnglePhi();

double quaternionToPlanar(geometry_msgs::Quaternion quaternion);
double distanceBetweenPoints(double x1, double y1, double x2, double y2);
double getDeltaPhi(double phi, double reference);

