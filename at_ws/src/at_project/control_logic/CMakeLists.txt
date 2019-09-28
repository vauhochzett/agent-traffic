cmake_minimum_required(VERSION 3.5.1)
project(control_logic)


find_package(catkin REQUIRED COMPONENTS
	message_generation
	geometry_msgs
	roscpp
	roslib
	std_msgs
	std_srvs
)

catkin_package(
  CATKIN_DEPENDS std_msgs std_srvs 
)

include_directories(${catkin_INCLUDE_DIRS})


