cmake_minimum_required(VERSION 2.8.3)
project(arc_utilities)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(DIRECTORY msg FILES )

## Generate services in the 'srv' folder
# add_service_files(DIRECTORY srv FILES )

## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES geometry_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs DEPENDS system_lib z Eigen)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -Werror")

# Utility library
add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/abb_irb1600_145_fk_fast.hpp
    include/${PROJECT_NAME}/arc_helpers.hpp
    include/${PROJECT_NAME}/pretty_print.hpp
    include/${PROJECT_NAME}/zlib_helpers.hpp
    include/${PROJECT_NAME}/voxel_grid.hpp
    include/${PROJECT_NAME}/dynamic_spatial_hashed_voxel_grid.hpp
    include/${PROJECT_NAME}/eigen_helpers.hpp
    include/${PROJECT_NAME}/eigen_helpers_conversions.hpp
    include/${PROJECT_NAME}/simple_rrt_planner.hpp
    src/${PROJECT_NAME}/zlib_helpers.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} z)

# Simple test node for voxel_grid
add_executable(test_arc_utilities src/test_arc_utilities.cpp)
add_dependencies(test_arc_utilities ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
target_link_libraries(test_arc_utilities ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark library for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
