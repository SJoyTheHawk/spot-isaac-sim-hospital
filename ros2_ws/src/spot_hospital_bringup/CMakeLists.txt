cmake_minimum_required(VERSION 3.8)
project(spot_hospital_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf maps launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
