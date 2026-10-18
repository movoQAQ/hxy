cmake_minimum_required(VERSION 3.8)
project(nav_executor)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
set(dependencies rclcpp rclcpp_action nav_task_interfaces map_server topology_global_planner
  geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs pluginlib)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

file(GLOB QPOASES_SOURCES third_party/qpOASES/src/*.cpp)
add_library(qpOASES_vendor STATIC ${QPOASES_SOURCES})
set_target_properties(qpOASES_vendor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(qpOASES_vendor PUBLIC third_party/qpOASES/include)
target_compile_definitions(qpOASES_vendor PRIVATE __NO_COPYRIGHT__ LINUX)

add_library(${PROJECT_NAME}_core src/task_manager.cpp src/path_planner.cpp src/path_executor.cpp
  src/mpc_controller.cpp src/motion_fsm.cpp src/safety_monitor.cpp src/recovery_controller.cpp
  src/route_monitor.cpp src/path_collision.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})
target_link_libraries(${PROJECT_NAME}_core Threads::Threads Eigen3::Eigen qpOASES_vendor)

add_executable(nav_executor_node src/nav_executor_node.cpp)
target_link_libraries(nav_executor_node ${PROJECT_NAME}_core)
target_link_libraries(nav_executor_node yaml-cpp)
ament_target_dependencies(nav_executor_node ${dependencies})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(nav_executor_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS ${PROJECT_NAME}_core nav_executor_node qpOASES_vendor
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES third_party/qpOASES/LICENSE.txt DESTINATION share/${PROJECT_NAME}/licenses)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_supervisor test/test_navigation_supervisor.cpp)
  target_link_libraries(test_navigation_supervisor ${PROJECT_NAME}_core)
  ament_target_dependencies(test_navigation_supervisor ${dependencies})
  ament_add_gtest(test_velocity_tf test/test_velocity_tf.cpp)
  target_link_libraries(test_velocity_tf ${PROJECT_NAME}_core)
  ament_target_dependencies(test_velocity_tf ${dependencies})
  add_executable(swerve_tracking_probe test/swerve_tracking_probe.cpp)
  target_link_libraries(swerve_tracking_probe ${PROJECT_NAME}_core)
  ament_target_dependencies(swerve_tracking_probe ${dependencies})
  ament_add_gtest(test_mpc_controller test/test_mpc_controller.cpp)
  target_link_libraries(test_mpc_controller ${PROJECT_NAME}_core)
  ament_target_dependencies(test_mpc_controller ${dependencies})
endif()

ament_package()
