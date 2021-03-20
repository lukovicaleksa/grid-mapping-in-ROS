cmake_minimum_required(VERSION 3.0.2)
project(grid_mapping)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs )

## Declare a catkin package
catkin_package()

# Install python scripts
catkin_install_python(PROGRAMS scripts/create_from_rosbag.py
				scripts/message_handler.py
				scripts/utils.py
				scripts/grid_map.py
				scripts/bresenham.py
				scripts/rtime_gmapping_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
