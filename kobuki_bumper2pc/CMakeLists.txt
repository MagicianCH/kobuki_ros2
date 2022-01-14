cmake_minimum_required(VERSION 3.5)
project(kobuki_bumper2pc)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/bumper_to_pc.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "kobuki_ros_interfaces"
)
rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "kobuki_bumper2pc::BumperToPC"
  EXECUTABLE ${PROJECT_NAME}_node)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
