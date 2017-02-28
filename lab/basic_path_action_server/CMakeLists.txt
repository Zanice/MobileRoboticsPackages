cmake_minimum_required(VERSION 2.8.3)
project(basic_path_action_server)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

cs_add_executable(bpas_alarm src/bpas_alarm.cpp)
cs_add_executable(bpas_client src/bpas_client.cpp)
cs_add_executable(bpas_server src/bpas_server.cpp)

add_library(twist_helper include/twist_helper/twist_helper.cpp)

target_link_libraries(bpas_client twist_helper)
target_link_libraries(bpas_server twist_helper)

install(TARGETS twist_helper LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/twist_helper/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

cs_install()
cs_export()

