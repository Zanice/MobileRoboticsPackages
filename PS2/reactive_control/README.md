##Reactive Cotnrol

####Nodes
dist_alarm (dist_alarm.cpp)
reactive_publisher (reactive_publisher.cpp)

####Description
The dist_alarm and reactive_publisher nodes work in tandom to allow an STDR robot to safely explore the area around it. The dist_alarm node subscribes to the robot's lidar system and publishes an alarm based on encountered obstacles. The ractive_publisher subscribes to this alarm and uses it to command the robot according to encountered obstacles.

####Notes
For use with "Server with Map and GUI plus Robot" STDR launcher.
