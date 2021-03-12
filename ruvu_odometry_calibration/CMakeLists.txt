cmake_minimum_required(VERSION 3.0.2)
project(ruvu_odometry_calibration)

find_package(catkin REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/calibrate
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
