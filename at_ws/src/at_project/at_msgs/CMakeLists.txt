cmake_minimum_required(VERSION 3.5.1)
project(at_msgs)


find_package(catkin REQUIRED COMPONENTS
	message_generation
	geometry_msgs
	roscpp
	roslib
	std_msgs
	std_srvs
)


add_service_files(
  FILES
  NewAgentSrv.srv
  NextMoveSrv.srv
  TurtlePosition.srv
)

add_message_files(
  FILES
  PositionMsg.msg
  ActionMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs std_srvs 
)

include_directories(${catkin_INCLUDE_DIRS})


