cmake_minimum_required(VERSION 3.8)
project(teb_nav2_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  maps
  meshes
  params
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pcd_to_nav2_map.py
  scripts/path_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
