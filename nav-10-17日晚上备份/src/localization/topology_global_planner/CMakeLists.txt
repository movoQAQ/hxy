
cmake_minimum_required(VERSION 3.8)
project(topology_global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(map_server REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Use the user's clone directly; do not build its GUI/demo or kinodynamic planner.
set(TDT_NAV_KIT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../tdt-nav-kit"
  CACHE PATH "Path to the tdt-nav-kit checkout")
if(EXISTS "${TDT_NAV_KIT_ROOT}/3rd/install")
  list(PREPEND CMAKE_PREFIX_PATH "${TDT_NAV_KIT_ROOT}/3rd/install")
endif()
find_package(OsqpEigen REQUIRED)
foreach(source src/YAstar/yastar.cpp src/MinimumSnapOsqp/minimumSnap.cpp
    src/MinimumSnapOsqp/sfcSquare.cpp)
  if(NOT EXISTS "${TDT_NAV_KIT_ROOT}/${source}")
    message(FATAL_ERROR "Missing ${source}; set TDT_NAV_KIT_ROOT to your tdt-nav-kit clone")
  endif()
endforeach()
add_library(tdt_nav_backend STATIC
  ${TDT_NAV_KIT_ROOT}/src/YAstar/yastar.cpp
  ${TDT_NAV_KIT_ROOT}/src/MinimumSnapOsqp/minimumSnap.cpp
  ${TDT_NAV_KIT_ROOT}/src/MinimumSnapOsqp/sfcSquare.cpp)
set_target_properties(tdt_nav_backend PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(tdt_nav_backend PRIVATE cxx_std_20)
target_include_directories(tdt_nav_backend PUBLIC
  ${TDT_NAV_KIT_ROOT}/src/YAstar ${TDT_NAV_KIT_ROOT}/src/MinimumSnapOsqp
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tdt_nav_backend PUBLIC Eigen3::Eigen OsqpEigen::OsqpEigen ${OpenCV_LIBS})

set(TOPO_LIB_NAME ${PROJECT_NAME}_core)

add_library(${TOPO_LIB_NAME} SHARED
  # Core
  src/planner_lifecycle.cpp
  src/path_planning.cpp
  # Topology
  src/topology/topology_io.cpp
  src/topology/topology_search.cpp
  src/topology/portal_optimizer.cpp
  src/topology/geometry_utils.cpp
  # Grid
  src/grid/grid_planner.cpp
  src/grid/safe_corner_cut.cpp
)

target_include_directories(${TOPO_LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${TOPO_LIB_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${TOPO_LIB_NAME}
  rclcpp
  map_server
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

target_link_libraries(${TOPO_LIB_NAME}
  yaml-cpp
  tdt_nav_backend
)

# Keep runtime dependencies beside the core library in the ROS install tree.
get_target_property(TDT_EIGEN_RUNTIME OsqpEigen::OsqpEigen LOCATION)
get_target_property(TDT_OSQP_RUNTIME osqp::osqp LOCATION)
install(FILES ${TDT_EIGEN_RUNTIME} ${TDT_OSQP_RUNTIME} DESTINATION lib)
install(FILES ${TDT_NAV_KIT_ROOT}/LICENCE
  DESTINATION share/${PROJECT_NAME} RENAME tdt-nav-kit-LICENCE)

if(EXISTS "${TDT_NAV_KIT_ROOT}/3rd/install/share/licenses")
  install(DIRECTORY ${TDT_NAV_KIT_ROOT}/3rd/install/share/licenses/
    DESTINATION share/${PROJECT_NAME}/third_party_licenses)
endif()

install(TARGETS
  ${TOPO_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${TOPO_LIB_NAME})

ament_export_dependencies(
  rclcpp
  map_server
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tdt_planner test/test_tdt_planner.cpp)
  target_link_libraries(test_tdt_planner ${TOPO_LIB_NAME})
  ament_target_dependencies(test_tdt_planner rclcpp map_server geometry_msgs nav_msgs)
endif()

ament_package()
