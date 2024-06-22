set(ROS_DRIVER_LIB ros_stim300_driver_lib)

add_library(
    ${ROS_DRIVER_LIB}
    SHARED    
)

target_sources(
    ${ROS_DRIVER_LIB}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_stim300_driver.cpp
)

ament_target_dependencies(
    ${ROS_DRIVER_LIB}
    PUBLIC
    rclcpp
    sensor_msgs
    std_srvs
)


target_include_directories(
    ${ROS_DRIVER_LIB}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

target_link_libraries(
  ${ROS_DRIVER_LIB}
  PUBLIC
  stim300_driver_lib
)

install(TARGETS
    ${ROS_DRIVER_LIB}
    DESTINATION lib
)


set(STIM300_DRIVER_BINARY_NAME ros_stim300_driver_node)

add_executable(
  ${STIM300_DRIVER_BINARY_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/ros_stim300_driver_node.cpp
)

target_include_directories(
  ${STIM300_DRIVER_BINARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

target_link_libraries(
    ${STIM300_DRIVER_BINARY_NAME}
    ${ROS_DRIVER_LIB}
)

install(
  TARGETS
  ${STIM300_DRIVER_BINARY_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
