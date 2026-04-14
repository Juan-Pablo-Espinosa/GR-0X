cmake_minimum_required(VERSION 3.8)
project(growbot_controllers)

find_package(ament_cmake REQUIRED)

foreach(dir IN ITEMS urdf meshes launch config rviz)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

ament_package()

