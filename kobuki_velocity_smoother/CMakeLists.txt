cmake_minimum_required(VERSION 3.5)
project(kobuki_velocity_smoother)

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(velocity_smoother SHARED src/velocity_smoother.cpp)
ament_target_dependencies(velocity_smoother
  "geometry_msgs"
  "nav_msgs"
  "rcl_interfaces"
  "rclcpp"
  "rclcpp_components"
)

add_executable(velocity_smoother_node src/velocity_smoother_node.cpp)
ament_target_dependencies(velocity_smoother_node
  "rclcpp"
)
target_link_libraries(velocity_smoother_node velocity_smoother)

rclcpp_components_register_nodes(velocity_smoother
  "velocity_smoother::VelocitySmoother")

install(TARGETS velocity_smoother
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS velocity_smoother_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
