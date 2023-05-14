cmake_minimum_required(VERSION 3.5)
project(inertiallabs_ins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(inertiallabs_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(
include
${rclcpp_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${tf2_INCLUDE_DIRS}
${action_msgs_INCLUDE_DIRS}
${dynamic_reconfigure_INCLUDE_DIRS}
${inertiallabs_msgs_INCLUDE_DIRS}
)

add_executable(il_ins
src/il_ins.cpp
src/ILDriver.cpp
src/UDDParser.cpp
src/SerialPort.cpp
src/NetClient.cpp
)

ament_target_dependencies(il_ins
rclcpp
geometry_msgs
sensor_msgs
std_msgs
tf2
action_msgs
dynamic_reconfigure
inertiallabs_msgs
)

install(TARGETS il_ins
DESTINATION lib/${PROJECT_NAME}
)

ament_package()