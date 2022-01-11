cmake_minimum_required(VERSION 3.0.2)
project(mission_planner)

add_compile_options(-std=c++11)

# catkin ws uses an out-of-source build setup
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
        roscpp
	sat_goal
	std_msgs
	tf
	tf2
	tf2_geometry_msgs
	tf2_ros
)

message(STATUS "All essential dependencies satisfied for ${PROJECT_NAME}")

catkin_package(
	INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${PROJECT_BINARY_DIR}
	LIBRARIES mission_planner
	CATKIN_DEPENDS roscpp sat_goal
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/mission.cpp
	src/node.cpp
	src/scheduler.cpp
	src/trail.cpp
	src/recorder.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_mission src/mission.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_mission
	PROPERTIES OUTPUT_NAME mission PREFIX ""
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_mission
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_mission
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

# TODO add install commands

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
#install(TARGETS ${PROJECT_NAME} mission
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS ${PROJECT_NAME} mission
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES
	#DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_trail.cpp)

if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
    )
endif()
