cmake_minimum_required(VERSION 3.5)
project(gril_calib)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++14 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)
  

add_executable(gril_calib_exec
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/Gril_Calib/Gril_Calib.cpp
  src/preprocess.cpp
  include/Fusion/FusionAhrs.c
  include/Fusion/FusionCompass.c
  include/Fusion/FusionOffset.c
)

ament_target_dependencies(gril_calib_exec
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_conversions
  livox_ros_driver2
  tf2_ros
  Eigen3
  PCL
  Ceres
)

target_link_libraries(gril_calib_exec
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${CERES_LIBRARIES}
)

target_include_directories(gril_calib_exec PRIVATE
  ${PYTHON_INCLUDE_DIRS}
)

rosidl_target_interfaces(gril_calib_exec ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS gril_calib_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()