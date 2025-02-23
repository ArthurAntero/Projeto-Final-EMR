cmake_minimum_required(VERSION 3.8)

project(projeto-final-pkg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME} geometry_msgs rclcpp std_msgs rclcpp_action nav2_msgs)

target_link_libraries(${PROJECT_NAME}
    rclcpp
    behaviortree_cpp_v3
    rclcpp_action
    nav2_msgs
    nav2_util
    nav2_core
    nav2_behavior_tree
    geometry_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})