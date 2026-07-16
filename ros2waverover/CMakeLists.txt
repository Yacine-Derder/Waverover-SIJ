cmake_minimum_required(VERSION 3.14)

project(ros2waverover LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core SerialPort REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core SerialPort REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(SRC
  src/source/main.cpp
  src/source/RobotController.cpp
  src/source/ROS2Subscriber.cpp
  src/source/UARTSerialPort.cpp
  src/include/RobotController.hpp
  src/include/UARTSerialPort.hpp
)

add_executable(ros2waverover-node ${SRC})

target_include_directories(ros2waverover-node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ros2waverover-node PUBLIC cxx_std_17)

install(TARGETS ros2waverover-node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/manual_lr_ui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_target_dependencies(ros2waverover-node rclcpp geometry_msgs sensor_msgs std_msgs)
target_link_libraries(ros2waverover-node Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::SerialPort)

ament_package()
