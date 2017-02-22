#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TwistCommander {
	private:
		ros::Publisher* twist_commander_;
		ros::Rate* loop_timer_;
		geometry_msgs::Twist* twist_cmd_;
		
	public:
		TwistCommander(ros::Publisher* twist_commander, ros::Rate* loop_timer, geometry_msgs::Twist* twist_cmd);
		
		void doSomething();
};

