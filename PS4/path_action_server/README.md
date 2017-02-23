##Path Action Server

####Nodes
collision_alarm (collision_alarm.cpp)
path_action_server (path_trace_service.cpp)
path_action_client (path_trace_client.cpp)

####Description
The path action client assembles a sequence of poses, each including a position and orientation, that the robot is to execute pose by pose up to the final pose. The path action server receives this sequence of nodes, processes the steps needed for completion for each pose, and publishes the necessary commands to the robot so that these poses are executed in sequence. Feedback is provided back to the client during this time. The alarm can be used in conjunction with the client, so that the current goal can be aborted if possible collision with an obstacle is detected. In such an event, the server returns the poses that have not been successfuly reached, and the client constructs a new path that reverts the path slightly and attempts to complete the remainder of the path.

####Notes
For use with "Server with Map and GUI plus Robot" STDR launcher. Requires included "stdr_twist".

