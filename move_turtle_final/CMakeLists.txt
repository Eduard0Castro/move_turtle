cmake_minimum_required(VERSION 3.8)
project(move_turtle_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(move_turtle_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(include)

add_library(turtle_control SHARED src/turtle_control.cpp)
ament_target_dependencies( turtle_control 
  rclcpp 
  turtlesim 
  rclcpp_lifecycle 
  rclcpp_action 
  move_turtle_interfaces
  geometry_msgs
  rclcpp_components

  )

add_library(turtle_bot_control SHARED src/turtle_bot_control.cpp)
ament_target_dependencies( turtle_bot_control
  rclcpp 
  rclcpp_lifecycle 
  rclcpp_action 
  move_turtle_interfaces
  geometry_msgs
  rclcpp_components
  )


rclcpp_components_register_nodes(turtle_control "TurtleFinal::TurtleControl")
rclcpp_components_register_nodes(turtle_bot_control "TurtleBotFinal::TurtleBotControl")

install (TARGETS 

  turtle_control
  turtle_bot_control

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

add_executable(turtle_manager src/turtle_lifecycle_manager.cpp)
ament_target_dependencies(turtle_manager rclcpp lifecycle_msgs)

install(TARGETS
  turtle_manager

  DESTINATION lib/${PROJECT_NAME}
)

install (

  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/

)

ament_package()
