cmake_minimum_required(VERSION 2.8.3)

project(mgbot3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy)

catkin_package()

find_package(roslaunch)


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})

endforeach(dir)

