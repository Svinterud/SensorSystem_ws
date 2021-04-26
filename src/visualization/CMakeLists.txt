cmake_minimum_required(VERSION 3.5)
project(visualization)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 
