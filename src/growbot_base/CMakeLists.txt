cmake_minimum_required(VERSION 3.8)
project(growbot_base)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(basic_command src/basic.cpp)
ament_target_dependencies(basic_command rclcpp trajectory_msgs)

install(TARGETS basic_command
  DESTINATION lib/${PROJECT_NAME})

foreach(dir IN ITEMS urdf meshes launch config rviz)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

#registers with gazebo
ament_environment_hooks("hooks/gz_resource_path.dsv.in")

ament_package()

