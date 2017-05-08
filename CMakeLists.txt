cmake_minimum_required(VERSION 2.8.3)
project(butter)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# BINARIES #
############

cs_add_executable(butter_node
  src/butter.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
