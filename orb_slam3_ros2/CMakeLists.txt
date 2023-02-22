cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DFMT_HEADER_ONLY")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# add_compile_options(-g)

set(ORB_SLAM3_PATH ~/src/ORB_SLAM3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(cart_interfaces REQUIRED)
# find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(GLEW QUIET)
# find_package(sensor_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)

set(INCLUDE_DIRS_ORBSLAM
    ${EIGEN3_INCLUDE_DIR}
    ${ORB_SLAM3_PATH}
    ${ORB_SLAM3_PATH}/include
    ${ORB_SLAM3_PATH}/include/CameraModels
    ${ORB_SLAM3_PATH}/Thirdparty/g2o
    ${ORB_SLAM3_PATH}/Thirdparty/Sophus
)

set(LIBS_ORBSLAM
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_PATH}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_PATH}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_PATH}/Thirdparty/g2o/lib/libg2o.so)

add_executable(orb_slam_mono src/mono.cpp)
target_include_directories(orb_slam_mono PUBLIC
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${ORB_SLAM3_PATH}
    ${ORB_SLAM3_PATH}/include
    ${ORB_SLAM3_PATH}/include/CameraModels
    ${ORB_SLAM3_PATH}/Thirdparty/g2o
    ${ORB_SLAM3_PATH}/Thirdparty/Sophus
)
target_link_libraries(orb_slam_mono
  ${LIBS_ORBSLAM})
ament_target_dependencies(orb_slam_mono
  rclcpp
  # std_msgs
  cv_bridge
  cart_interfaces
  # image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  # sensor_msgs
  # nav_msgs
  # orb_slam3_interfaces
)
target_compile_features(orb_slam_mono PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS orb_slam_mono
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
