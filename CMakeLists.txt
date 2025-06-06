cmake_minimum_required(VERSION 3.8)
project(octo_arm_teleop_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(stm_client REQUIRED)

set(dependencies
   stm_client
   ament_cmake
)


# add_library(pid
#   src/pid.cpp
# )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  launch
  "utapi"
  ${CMAKE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_subdirectory(utapi utapi)

add_executable(octo_arm_teleop_node src/octo_arm_teleop_node.cpp src/pid.cpp)
ament_target_dependencies(octo_arm_teleop_node ${CMAKE_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${CMAKE_EXPORTED_TARGETS} rclcpp std_msgs std_srvs geometry_msgs stm_client sensor_msgs)
target_link_libraries(octo_arm_teleop_node
        ${CMAKE_LIBRARIES}
        utapi)

target_include_directories(octo_arm_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(octo_arm_teleop_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS octo_arm_teleop_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include utapi launch
  DESTINATION share/${PROJECT_NAME}
)  

rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/Sendadrastatus.msg"
   "srv/GUIadrastat.srv"
   "srv/SetIkMode.srv"
   
   DEPENDENCIES geometry_msgs std_msgs

)




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

ament_package()
