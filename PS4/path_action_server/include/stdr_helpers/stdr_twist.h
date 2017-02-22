#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
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
		template <class T> double performTwist(double duration, actionlib::SimpleActionServer<T>* sas);
		double performStationary(double duration);
		template <class T> double performStationary(double duration, actionlib::SimpleActionServer<T>* sas);
		double performForward(double distance);
		template <class T> double performForward(double distance, actionlib::SimpleActionServer<T>* sas);
		double performTurn(int direction, double radians);
		template <class T> double performTurn(int direction, double radians, actionlib::SimpleActionServer<T>* sas);
		double performRightAngleTurn(int direction);
		template <class T> double performRightAngleTurn(int direction, actionlib::SimpleActionServer<T>* sas);
		
	public:
		TwistCommander(ros::Publisher* twist_commander, ros::Rate* loop_timer, double dt);
		
		void configureTwistParameters(double move_speed, double turn_speed, double trans_time);
		
		double cmdStationary(double duration);
		template <class T> double cmdStationary(double duration, actionlib::SimpleActionServer<T>* sas);
		double cmdForward(double distance);
		template <class T> double cmdForward(double distance, actionlib::SimpleActionServer<T>* sas);
		double cmdTurn(int direction, double radians);
		template <class T> double cmdTurn(int direction, double radians, actionlib::SimpleActionServer<T>* sas);
		double cmdRightAngleTurn(int direction);
		template <class T> double cmdRightAngleTurn(int direction, actionlib::SimpleActionServer<T>* sas);
};

double quaternionToPlanar(geometry_msgs::Quaternion quaternion);
double distanceBetweenPoints(double x1, double y1, double x2, double y2);
double getDeltaPhi(double phi, double reference);

