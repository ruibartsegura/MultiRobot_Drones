cmake_minimum_required(VERSION 3.5)
project(reynold_rules)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(dependencies
    rclcpp
    std_msgs
    geometry_msgs
    reynold_rules_interfaces
)

# Using a library
include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/reynold_rules_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(reynold_rules_main src/reynold_rules_main.cpp)
ament_target_dependencies(reynold_rules_main ${dependencies})
target_link_libraries(reynold_rules_main ${PROJECT_NAME})

install(TARGETS
  reynold_rules_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
