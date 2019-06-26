cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand_parameters)

find_package(catkin REQUIRED)

# Declare a package.
catkin_package()

#############
## Install ##
#############

## Install all the parameter files
install(DIRECTORY .
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  FILES_MATCHING PATTERN "*.yaml"
)
