cmake_minimum_required(VERSION 2.8.9)
project(ground_station)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs sensor_msgs cv_bridge)
find_package(Qt5Core)
find_package(OpenCV REQUIRED COMPONENTS highgui imgproc)

set(CMAKE_BUILD_TYPE RelWithDebugInfo)
set(CMAKE_CXX_FLAGS, "-std=c++11")

catkin_package(INCLUDE_DIRS include)
include_directories(include)

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/ground_station/*.h)
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/ground_station/*.cpp)

add_executable(ground_station_node ${QT_SOURCES} ${QT_MOC_HPP})

add_dependencies(ground_station_node ${PROJECT_NAME}_generate_messages_cpp)
qt5_use_modules(ground_station_node Core)

target_link_libraries(ground_station_node ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OpenCV_LIBS})