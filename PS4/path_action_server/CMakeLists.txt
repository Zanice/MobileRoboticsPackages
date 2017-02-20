cmake_minimum_required(VERSION 2.8.3)
project(path_action_server)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

cs_add_executable(collision_alarm src/collision_alarm.cpp)
cs_add_executable(path_action_client src/path_action_client.cpp)
cs_add_executable(path_action_server src/path_action_server.cpp)

cs_install()
cs_export()
    
