cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  allegro_hand_driver
  bhand
  libpcan
  roscpp
  sensor_msgs
# roslaunch  # Disabled due to testing
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES allegro_hand_controllers
  CATKIN_DEPENDS allegro_hand_driver bhand libpcan roscpp sensor_msgs
#  DEPENDS
)

###########
## Build ##
###########
message( ${catkin_INCLUDE_DIRS} )
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare executables
#add_library(allegro_hand_node src/allegro_hand_node.cpp)

#add_executable(allegro_hand_grasp src/allegro_hand_grasp.cpp)
#target_link_libraries(allegro_hand_grasp allegro_hand_node ${catkin_LIBRARIES})

add_executable(allegro_hand_uname src/allegro_hand_uname.cpp)
target_link_libraries(allegro_hand_uname ${catkin_LIBRARIES})

#add_executable(allegro_hand_viz src/allegro_hand_visualizer.cpp)
#target_link_libraries(allegro_hand_viz allegro_hand_node ${catkin_LIBRARIES})

#add_executable(allegro_hand_joint_pd src/allegro_hand_joint_pd.cpp)
#target_link_libraries(allegro_hand_joint_pd allegro_hand_node ${catkin_LIBRARIES})

#add_executable(allegro_hand_joint_velsat src/allegro_hand_joint_velsat.cpp)
#target_link_libraries(allegro_hand_joint_velsat allegro_hand_node ${catkin_LIBRARIES})

#add_executable(allegro_hand_joint_torque src/allegro_hand_joint_torque.cpp)
#target_link_libraries(allegro_hand_joint_torque allegro_hand_node ${catkin_LIBRARIES})

# This is disabled until roslaunch tests can pass required arguments.
# roslaunch_add_file_check(launch/allegro_hand.launch)

#############
## Install ##
#############

install(TARGETS
  #allegro_hand_grasp
  allegro_hand_uname
  #allegro_hand_viz
  #allegro_hand_joint_pd
  #allegro_hand_joint_torque
  #allegro_hand_joint_velsat
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  FILES_MATCHING PATTERN "*.launch"
)
