cmake_minimum_required(VERSION 2.8.3)

project(volti)

find_package(
	catkin 
	REQUIRED COMPONENTS 
		roscpp 
		rospy 
		std_msgs 
		tf 
		joy 
		message_generation 
		dynamic_reconfigure
		volti_msgs
)

#set(ROS_BUILD_TYPE Release)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

generate_dynamic_reconfigure_options(
    cfg/VOL.cfg
)

add_message_files( FILES float32_12.msg float32_3.msg pid_ext.msg)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	INCLUDE_DIRS include
    CATKIN_DEPENDS 
    	message_runtime
    	roscpp 
    	std_msgs
	volti_msgs
)

include_directories(
	include 
	include/imu_lib
	${catkin_INCLUDE_DIRS}
)

add_library(ADXL345
	src/ADXL345.cpp
)
target_link_libraries(ADXL345 ${catkin_LIBRARIES})

add_library(gyro
	src/gyro.cpp
)
target_link_libraries(gyro ${catkin_LIBRARIES})

add_library(hmc5883
	src/hmc5883.cpp
)
target_link_libraries(hmc5883 ${catkin_LIBRARIES})

add_library(imu_lib
	src/imu_lib.cpp
)
target_link_libraries(imu_lib ADXL345 gyro hmc5883 ${catkin_LIBRARIES})

add_executable(imu_node
	nodes/imu_node.cpp
)
target_link_libraries(imu_node imu_lib ${catkin_LIBRARIES})

add_executable(imu_plate
	nodes/imu_plate.cpp
)
target_link_libraries(imu_plate imu_lib ${catkin_LIBRARIES})

add_executable(imu_pendu
	nodes/imu_pendu.cpp
)
target_link_libraries(imu_pendu imu_lib ${catkin_LIBRARIES})