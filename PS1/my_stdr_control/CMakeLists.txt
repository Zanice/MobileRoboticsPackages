cmake_minimum_required(VERSION 2.8.3)
project(my_stdr_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(zanice_stdr_libs)


catkin_package(
  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${zanice_stdr_libs_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(my_stdr_publisher src/my_stdr_publisher.cpp)
add_executable(my_stdr_publisher_remake src/my_stdr_publisher_remake.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(my_stdr_publisher ${catkin_LIBRARIES})
target_link_libraries(my_stdr_publisher_remake ${catkin_LIBRARIES})
target_link_libraries(my_stdr_publisher_remake zanice_stdr_libs)

