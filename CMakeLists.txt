cmake_minimum_required(VERSION 2.8.3)

project(vaccumbot)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	std_msgs
	geometry_msgs
	urdf
	xacro
	message_generation
	sensor_msgs
	gazebo_ros
	nav_msgs
	tf
	)

catkin_package(
	CATKIN_DEPENDS
	geometry_msgs
	roscpp
	rospy
	std_msgs
	sensor_msgs
	gazebo_ros
	nav_msgs
	tf
	urdf
	xacro
	)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
