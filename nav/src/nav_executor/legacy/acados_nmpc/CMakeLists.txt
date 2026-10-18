cmake_minimum_required(VERSION 3.8)
project(nav_executor LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
set(dependencies rclcpp map_server topology_global_planner
  geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_ros rcl_interfaces tf2_geometry_msgs pluginlib)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

# The generated source and Acados runtime must use the same locked revision.
set(ACADOS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.cache/acados" CACHE PATH "Locked Acados installation")
if(DEFINED ENV{ACADOS_INSTALL_DIR})
  set(ACADOS_ROOT "$ENV{ACADOS_INSTALL_DIR}")
endif()
if(NOT EXISTS "${ACADOS_ROOT}/include/acados_c/ocp_nlp_interface.h")
  message(FATAL_ERROR "Acados missing. Run scripts/setup_acados.sh or set ACADOS_ROOT.")
endif()
if(NOT EXISTS "${ACADOS_ROOT}/lib/git_commit_hash")
  message(FATAL_ERROR "Acados revision marker missing; use scripts/setup_acados.sh.")
endif()
file(READ "${ACADOS_ROOT}/lib/git_commit_hash" ACADOS_REVISION)
string(STRIP "${ACADOS_REVISION}" ACADOS_REVISION)
if(NOT ACADOS_REVISION STREQUAL "8ff7c2a17e235d6cad6671538e923ccb7428dc3e")
  message(FATAL_ERROR "Acados revision differs from the checked-in NMPC solver.")
endif()
file(GLOB_RECURSE OMNI_GENERATED CONFIGURE_DEPENDS generated/*.c)
if(NOT OMNI_GENERATED)
  message(FATAL_ERROR "NMPC generated sources missing. Run scripts/generate_omni_solver.py.")
endif()
add_library(omni_mpc_solver STATIC ${OMNI_GENERATED})
set_target_properties(omni_mpc_solver PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(omni_mpc_solver PUBLIC generated
  "${ACADOS_ROOT}/include" "${ACADOS_ROOT}/include/blasfeo/include"
  "${ACADOS_ROOT}/include/hpipm/include")
find_library(ACADOS_LIBRARY acados PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(HPIPM_LIBRARY hpipm PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(BLASFEO_LIBRARY blasfeo PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
target_link_libraries(omni_mpc_solver PUBLIC ${ACADOS_LIBRARY} ${HPIPM_LIBRARY} ${BLASFEO_LIBRARY} m)
# Ship matching runtime libraries; no dependency on the build machine's cache at launch.
install(FILES ${ACADOS_LIBRARY} ${HPIPM_LIBRARY} ${BLASFEO_LIBRARY} DESTINATION lib)
# BLASFEO has a versioned SONAME.
file(GLOB BLASFEO_RUNTIME "${ACADOS_ROOT}/lib/libblasfeo.so.*")
install(FILES ${BLASFEO_RUNTIME} DESTINATION lib)
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/..")

add_library(${PROJECT_NAME}_core src/task_manager.cpp src/path_planner.cpp src/path_executor.cpp
  src/mpc_controller.cpp src/mpc/geometry.cpp
  src/route_monitor.cpp src/path_collision.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})
target_link_libraries(${PROJECT_NAME}_core Threads::Threads Eigen3::Eigen omni_mpc_solver)

add_executable(nav_executor_node src/nav_executor_node.cpp)
target_link_libraries(nav_executor_node ${PROJECT_NAME}_core)
target_link_libraries(nav_executor_node yaml-cpp)
ament_target_dependencies(nav_executor_node ${dependencies})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(nav_executor_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS ${PROJECT_NAME}_core nav_executor_node omni_mpc_solver
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES third_party/mpc_planner/LICENSE DESTINATION share/${PROJECT_NAME}/licenses/mpc_planner)
install(DIRECTORY third_party/acados/ DESTINATION share/${PROJECT_NAME}/licenses/acados)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_velocity_tf test/test_velocity_tf.cpp)
  target_link_libraries(test_velocity_tf ${PROJECT_NAME}_core)
  ament_target_dependencies(test_velocity_tf ${dependencies})
  ament_add_gtest(test_nmpc_controller test/test_nmpc_controller.cpp)
  target_link_libraries(test_nmpc_controller ${PROJECT_NAME}_core)
  ament_target_dependencies(test_nmpc_controller ${dependencies})
endif()

ament_package()
