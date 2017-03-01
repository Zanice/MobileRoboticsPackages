#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <zanice_stdr_libs/twist_helper.h>

// - - - - - - - - - - - - - - -
// CONSTANT AND GLOBAL VARIABLES
// - - - - - - - - - - - - - - -

// node name
const char* NODE_NAME = "my_stdr_commander_remake";
// name of velocity topic to publish to
const char* VEL_TOPIC_NAME = "/robot0/cmd_vel";

// robot warm-up time
const double WARM_UP_TIME = 0.2;
// robot movement speed value in m/s
const double MOVE_SPEED = 1.0;
// robot turn speed value in rads/s
const double TURN_SPEED = 0.5;
// robot command transition time
const double TRANS_TIME = 0.1;

// update rate
const double DT = 0.01;

// movement speed correction factor
const double MOVE_SPEED_FACTOR = 1.0;
// movement speed correction buffer
const double MOVE_SPEED_BUFFER = 0.0;
// turn speed correction factor
const double TURN_SPEED_FACTOR = 1.0;
// turn speed correction buffer
const double TURN_SPEED_BUFFER = 0.0;

// - - - - - - - - - -
// CLASSES AND METHODS
// - - - - - - - - - -

void stationary(TwistCommander* commander, double duration) {
	ROS_INFO("Being stationary for %f seconds.", duration);
	double remaining = commander->cmdStationary(duration);
	ROS_WARN("\t%f remaining after twist.", remaining);
}

void stationaryIter(TwistCommander* commander, double duration) {
	ROS_INFO("Being stationary for %f seconds.", duration);
	double remaining = duration;
	while (remaining > 0) {
		remaining = commander->cmdStationaryIter(remaining);
	}
	ROS_WARN("\t%f remaining after twist.", remaining);
}

void forward(TwistCommander* commander, double distance) {
	ROS_INFO("Moving forward %f meters.", distance);
	double remaining = commander->cmdForward(distance);
	ROS_WARN("\t%f remaining after twist.", remaining);
}

void forwardIter(TwistCommander* commander, double distance) {
	ROS_INFO("Moving forward %f meters.", distance);
	double remaining = distance;
	while (remaining > 0) {
		remaining = commander->cmdForwardIter(remaining);
	}
	ROS_WARN("\t%f remaining after twist.", remaining);
}

void turn(TwistCommander* commander, int direction, double phi) {
	ROS_INFO("Turning %f radians in direction %d.", phi, direction);
	double remaining = commander->cmdTurn(direction, phi);
	ROS_WARN("\t%f remaining after twist.", remaining);
}

void turnIter(TwistCommander* commander, int direction, double phi) {
	ROS_INFO("Turning %f radians in direction %d.", phi, direction);
	double remaining = phi;
	while (remaining > 0) {
		remaining = commander->cmdTurnIter(direction, remaining);
	}
	ROS_WARN("\t%f remaining after twist.", remaining);
}

// - - - - - -
// MAIN METHOD
// - - - - - -

int main(int argc, char **argv) {
	// initialize the ROS publisher
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;
	
	// set up robot twist commander
	ros::Publisher twist_commander = node.advertise<geometry_msgs::Twist>(VEL_TOPIC_NAME, 1);
	ros::Rate loop_timer(1 / DT);
	TwistCommander commander(&twist_commander, &loop_timer, DT);
	
	// configure parameters for robot twist commander
	commander.configureTwistParameters(MOVE_SPEED, TURN_SPEED, TRANS_TIME);
	commander.configureTwistFactors(MOVE_SPEED_FACTOR, TURN_SPEED_FACTOR);
	commander.configureTwistBuffers(MOVE_SPEED_BUFFER, TURN_SPEED_BUFFER);
	
	// warm up communication with stationary command
	stationary(&commander, WARM_UP_TIME);
	
	// MAIN PROCESS: Make some moves
	forward(&commander, 2.5);
	turn(&commander, 1, rightAnglePhi());
	forward(&commander, 1.0);
	
	// MAIN PROCESS: Make some moves, iteratively
	stationaryIter(&commander, WARM_UP_TIME);
	turnIter(&commander, -1, rightAnglePhi() * 2);
	forwardIter(&commander, 1.0);
	turnIter(&commander, -1, rightAnglePhi());
	forwardIter(&commander, 2.5);
	
	// perform stationary to ensure halt
	stationary(&commander, WARM_UP_TIME);
}

