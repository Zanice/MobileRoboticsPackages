cmake_minimum_required(VERSION 2.8.3)
project(path_action_server)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

cs_add_executable(collision_alarm src/collision_alarm.cpp)
cs_add_executable(path_action_client src/path_action_client.cpp)
cs_add_executable(path_action_server src/path_action_server.cpp)

add_library(stdr_twist include/stdr_helpers/stdr_twist.cpp)

target_link_libraries(path_action_client stdr_twist)
target_link_libraries(path_action_server stdr_twist)

install(TARGETS stdr_twist LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/stdr_helpers/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

cs_install()
cs_export()
