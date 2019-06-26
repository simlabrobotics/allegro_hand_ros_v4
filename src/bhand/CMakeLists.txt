cmake_minimum_required(VERSION 2.8.3)
project(bhand)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BHand
)

###########
## Build ##
###########

# NOTE: You can switch to 32 bit if necessary by updating the symbolic link.
find_library(BHAND_LIB
    NAMES BHand
    PATHS lib
)

# Make a tricky BHand target that just copies the .so binary file into the
# output directory.
add_library(BHand ${BHAND_LIB})
add_custom_command(TARGET BHand
POST_BUILD COMMAND cp ${BHAND_LIB} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ )
set_target_properties(BHand PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

## Mark header files for installation
install(DIRECTORY include/bhand/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS BHand
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
