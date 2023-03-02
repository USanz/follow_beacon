cmake_minimum_required(VERSION 3.5)
project(follow_beacon)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(cv_bridge REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
#  include
#  ${GAZEBO_INCLUDE_DIRS}
)


#set(EXEC_COLOR_FILTER_NAME "image_color_filter")
#set(EXEC_OP_NAME "operator_image")
#set(EXEC_SINK_NAME "sink_motors")
set(EXEC_CAM_PUBLISHER_NAME "camera_image_pub")
#set(EXEC_QR_CODE_DETECTOR_NAME "qr_code_detector")

set(dependencies
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "rclcpp"
  "OpenCV"
  "cv_bridge"
)

#add_executable(${EXEC_COLOR_FILTER_NAME} src/image_color_filter.cpp)
#ament_target_dependencies(${EXEC_COLOR_FILTER_NAME} ${dependencies})
#target_include_directories(${EXEC_COLOR_FILTER_NAME} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_link_libraries(${EXEC_COLOR_FILTER_NAME}
#  ${OpenCV_LIBRARIES})
#
#add_executable(${EXEC_OP_NAME} src/operator_image.cpp)
#ament_target_dependencies(${EXEC_OP_NAME} ${dependencies})
#target_include_directories(${EXEC_OP_NAME} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_link_libraries(${EXEC_OP_NAME}
#  ${OpenCV_LIBRARIES})
#
#add_executable(${EXEC_SINK_NAME} src/sink_motors.cpp)
#ament_target_dependencies(${EXEC_SINK_NAME} ${dependencies})
#target_include_directories(${EXEC_SINK_NAME} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

add_executable(${EXEC_CAM_PUBLISHER_NAME} src/camera_image_pub.cpp)
ament_target_dependencies(${EXEC_CAM_PUBLISHER_NAME} ${dependencies})
target_include_directories(${EXEC_CAM_PUBLISHER_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${EXEC_CAM_PUBLISHER_NAME}
  ${OpenCV_LIBRARIES})

#add_executable(${EXEC_QR_CODE_DETECTOR_NAME} src/qr_code_detector.cpp)
#ament_target_dependencies(${EXEC_QR_CODE_DETECTOR_NAME} ${dependencies})
#target_include_directories(${EXEC_QR_CODE_DETECTOR_NAME} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_link_libraries(${EXEC_QR_CODE_DETECTOR_NAME}
#  ${OpenCV_LIBRARIES})



install(TARGETS
#  ${EXEC_COLOR_FILTER_NAME}
#  ${EXEC_OP_NAME}
#  ${EXEC_SINK_NAME}
  ${EXEC_CAM_PUBLISHER_NAME}
#  ${EXEC_QR_CODE_DETECTOR_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

#install(DIRECTORY
#  launch/
#  DESTINATION share/${PROJECT_NAME}
#)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
