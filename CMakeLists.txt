cmake_minimum_required(VERSION 3.10.0)
project(arm_utilities)

set(CMAKE_CXX_STANDARD 20)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        ament_cmake_python
        rclcpp
        rclpy
        std_msgs
        pybind11
        pybind11_vendor
        sensor_msgs
        geometry_msgs
        trajectory_msgs
        Eigen3
)

foreach(THIS_PACKAGE_INCLUDE_DEPEND ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${THIS_PACKAGE_INCLUDE_DEPEND} REQUIRED)
endforeach()

pybind11_add_module(rclcpp_initializer src/rclcpp_initializer.cpp)
target_link_libraries(rclcpp_initializer PUBLIC rclcpp::rclcpp)

add_library(arm_utilities
    src/ros_helpers.cpp)
target_include_directories(arm_utilities
    PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
    arm_utilities
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


install(
        TARGETS arm_utilities rclcpp_initializer
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
)

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(arm_utilities)
ament_export_targets(${PROJECT_NAME}_targets)

ament_package()