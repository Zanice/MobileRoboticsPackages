cmake_minimum_required(VERSION 2.8.3)
project(zanice_stdr_libs)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

add_library(zanice_stdr_libs include/zanice_stdr_libs/twist_helper.cpp)

install(TARGETS zanice_stdr_libs LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/zanice_stdr_libs/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

cs_install()
cs_export()

