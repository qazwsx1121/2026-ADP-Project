cmake_minimum_required(VERSION 3.13)
project(svg_mppi)

# ---------------------------------------------------------------------------
# 알고리즘 본체는 공식 저장소(kohonda/proj-svg_mppi)의 파일을 "그대로" 컴파일한다.
# 공식 코드를 고치면(VS Code) 이 패키지를 다시 빌드하는 것만으로 반영된다.
#   기본 경로: ~/shared_dir/proj-svg_mppi/src/mppi_controller
#   다른 곳에 받았으면: colcon build ... --cmake-args -DSVG_MPPI_CORE_DIR=<경로>
# ---------------------------------------------------------------------------
set(SVG_MPPI_CORE_DIR "$ENV{HOME}/shared_dir/proj-svg_mppi/src/mppi_controller"
    CACHE PATH "official proj-svg_mppi/src/mppi_controller directory")
if(NOT EXISTS "${SVG_MPPI_CORE_DIR}/src/stein_variational_guided_mppi.cpp")
  message(FATAL_ERROR
    "\n[svg_mppi] 공식 코드를 찾을 수 없습니다: ${SVG_MPPI_CORE_DIR}\n"
    "  cd ~/shared_dir && git clone https://github.com/kohonda/proj-svg_mppi.git\n"
    "  또는 --cmake-args -DSVG_MPPI_CORE_DIR=<proj-svg_mppi/src/mppi_controller 경로>\n")
endif()
message(STATUS "[svg_mppi] official core: ${SVG_MPPI_CORE_DIR}")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# --- official solver (unmodified sources) ---
add_library(svg_mppi_solver SHARED
  ${SVG_MPPI_CORE_DIR}/src/prior_samples_with_costs.cpp
  ${SVG_MPPI_CORE_DIR}/src/mpc_base.cpp
  ${SVG_MPPI_CORE_DIR}/src/forward_mppi.cpp
  ${SVG_MPPI_CORE_DIR}/src/reverse_mppi.cpp
  ${SVG_MPPI_CORE_DIR}/src/stein_variational_mpc.cpp
  ${SVG_MPPI_CORE_DIR}/src/stein_variational_guided_mppi.cpp
)
target_include_directories(svg_mppi_solver PUBLIC
  ${SVG_MPPI_CORE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(svg_mppi_solver grid_map_core)
target_link_libraries(svg_mppi_solver OpenMP::OpenMP_CXX)

# --- ROS 2 node (wrapper for the ForzaETH race stack) ---
add_executable(svg_mppi_node src/svg_mppi_node.cpp)
target_include_directories(svg_mppi_node PRIVATE
  ${SVG_MPPI_CORE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(svg_mppi_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  ackermann_msgs f110_msgs tf2 tf2_ros grid_map_core)
target_link_libraries(svg_mppi_node svg_mppi_solver OpenMP::OpenMP_CXX)

install(TARGETS svg_mppi_solver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(TARGETS svg_mppi_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
