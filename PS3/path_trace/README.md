##Path Trace

####Nodes
path_trace_service (path_trace_service.cpp)
path_trace_client (path_trace_client.cpp)

####Description
The path trace client assembles a sequence of poses, each including a position and orientation, that the robot is to execute pose by pose up to the final pose. The path trace service receives this sequence of nodes, processes the steps needed for completion for each pose, and publishes the necessary commands to the robot so that these poses are executed in sequence.

####Notes
For use with "Server with Map and GUI plus Robot" STDR launcher.

