cmake_minimum_required(VERSION 3.0.2)
project(module_8)
set(CMAKE_CXX_STANDARD 17)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(PCL 1.8 REQUIRED)

# указываем в проекте все директории с заголовочными файлами
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include  # Указываем папку с заголовочными файлами
#  Перечисляем зависимости от других catkin-пакетов
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    std_msgs
)



## Объявляем библиотеки C++

#библиотека для управления БЛА
add_library(my_new_control_lib STATIC src/flight_controller.cpp)

#библиотека для управления лидаром
add_library(lidar_data_control STATIC src/lidar_control.cpp)

#библиотека для управления БЛА в режиме облета препятствий
add_library(aviodance_lib STATIC src/test_avoidance.cpp)

#библиотека для облета препятствий
add_library(vfh2d_lib STATIC src/vfh_test.cpp)

#библиотека для визуализации карты
add_library(visual_ STATIC src/map_viz.cpp)


# добавляем исполняемые файлы
#новая нода для управления БЛА
add_executable(avoid_flight_controller_node src/flight_controller_node.cpp)

#новая нода для управления БЛА
add_executable(avoidance_control_node src/avoidance_node.cpp)

#сборка нод для запуска и управления БЛА
target_link_libraries(my_new_control_lib 
  ${catkin_LIBRARIES}
)
#сборка ноды для запуска и управления БЛА
target_link_libraries(avoid_flight_controller_node
  ${catkin_LIBRARIES}
  my_new_control_lib
  lidar_data_control
  ${PCL_LIBRARIES}           # Основные библиотеки PCL
  ${PCL_COMMON_LIBRARIES}     # PCL common
  ${PCL_IO_LIBRARIES}        # PCL I/O
  ${PCL_VISUALIZATION_LIBRARIES}  # PCL visualization
) 

#сборка нод для запуска системы облета препятствий
target_link_libraries(aviodance_lib 
  ${catkin_LIBRARIES}
)

#сборка ноды для управления системой облета препятствий
target_link_libraries(avoidance_control_node
  ${catkin_LIBRARIES}
  aviodance_lib
  lidar_data_control
  my_new_control_lib
  vfh2d_lib
  visual_
  ${PCL_LIBRARIES}           # Основные библиотеки PCL
  ${PCL_COMMON_LIBRARIES}     # PCL common
  ${PCL_IO_LIBRARIES}        # PCL I/O
  ${PCL_VISUALIZATION_LIBRARIES}  # PCL visualization
) 
