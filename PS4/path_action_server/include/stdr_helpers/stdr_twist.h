#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

class TwistCommander {
	private:
		double MOVE_SPEED_;
		double TURN_SPEED_;
		double TRANS_TIME_;
		
		ros::Publisher* twist_commander_;
		ros::Rate* loop_timer_;
		double dt_;
		geometry_msgs::Twist* twist_cmd_;
		
		double performTwist(double duration);
		double performStationary(double duration);
		double performForward(double distance);
		double performTurn(int direction, double radians);
		double performRightAngleTurn(int direction);
		
	public:
		TwistCommander(ros::Publisher* twist_commander, ros::Rate* loop_timer, double dt);
		
		void configureTwistParameters(double move_speed, double turn_speed, double trans_time);
		
		double cmdStationary(double duration);
		double cmdForward(double distance);
		double cmdTurn(int direction, double radians);
		double cmdRightAngleTurn(int direction);
};

double quaternionToPlanar(geometry_msgs::Quaternion quaternion);
double distanceBetweenPoints(double x1, double y1, double x2, double y2);
double getDeltaPhi(double phi, double reference);

