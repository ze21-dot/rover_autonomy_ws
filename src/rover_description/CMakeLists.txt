cmake_minimum_required(VERSION 3.16)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz meshes worlds
        DESTINATION share/${PROJECT_NAME})

ament_package()
