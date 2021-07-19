cmake_minimum_required(VERSION 3.0.2)
project(ruvu_odometry_calibration)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  sensor_msgs
  tf2_ros
)
find_package(Ceres REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDES}
)

add_executable(calibrate++
  src/bag_buffer.cpp
  src/main.cpp
)
target_link_libraries(calibrate++
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(ceres_example src/ceres_example.cpp)
target_link_libraries(ceres_example
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS calibrate++ ceres_example
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
    scripts/calibrate
    scripts/plot_path
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
