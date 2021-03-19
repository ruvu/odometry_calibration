cmake_minimum_required(VERSION 3.0.2)
project(ruvu_odometry_calibration)

find_package(catkin REQUIRED)
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

add_executable(calibrate++ src/main.cpp)
target_link_libraries(calibrate++
  ${CERES_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS calibrate++
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
