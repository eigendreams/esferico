cmake_minimum_required(VERSION 2.8.3)

project(volti_msgs)

find_package(
	catkin 
	REQUIRED COMPONENTS 
		roscpp 
		rospy 
		std_msgs 
		tf 
		joy 
		message_generation 
)

#set(ROS_BUILD_TYPE Release)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_message_files( FILES float32_12.msg float32_3.msg pid_ext.msg)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    	message_runtime
    	roscpp 
    	std_msgs
)
