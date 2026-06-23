cmake_minimum_required(VERSION 3.8)
project(lio_sam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(lio_sam_dependencies
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
)

set(lio_sam_offline_dependencies
  ${lio_sam_dependencies}
  rosbag2_cpp
  rosbag2_storage
)

function(configure_lio_sam_target target_name)
  target_include_directories(${target_name} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

  target_link_libraries(${target_name}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    gtsam
    ${cpp_typesupport_target}
  )

  if(TARGET gtsam_unstable)
    target_link_libraries(${target_name} gtsam_unstable)
  elseif(EXISTS "/usr/lib/x86_64-linux-gnu/libgtsam_unstable.so")
    target_link_libraries(${target_name} gtsam_unstable)
  endif()

  if(OpenMP_CXX_FOUND)
    target_link_libraries(${target_name} OpenMP::OpenMP_CXX)
  endif()

  target_compile_features(${target_name} PUBLIC cxx_std_17)
endfunction()

function(add_lio_sam_online_node target_name source_file)
  add_executable(${target_name} ${source_file})
  ament_target_dependencies(${target_name} ${lio_sam_dependencies})
  configure_lio_sam_target(${target_name})
  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_lio_sam_online_node(lio_sam_imageProjection src/imageProjection.cpp)
add_lio_sam_online_node(lio_sam_featureExtraction src/featureExtraction.cpp)
add_lio_sam_online_node(lio_sam_imuPreintegration src/imuPreintegration.cpp)
add_lio_sam_online_node(lio_sam_mapOptimization src/mapOptmization.cpp)

add_executable(lio_sam_run_offline
  src/run_lio_sam_offline.cpp
  src/offline/bag_io.cpp
)
ament_target_dependencies(lio_sam_run_offline ${lio_sam_offline_dependencies})
configure_lio_sam_target(lio_sam_run_offline)
target_compile_definitions(lio_sam_run_offline PRIVATE LIO_SAM_OFFLINE_LIBRARY)
install(TARGETS lio_sam_run_offline
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
