cmake_minimum_required(VERSION 3.8)
project(gnss_cam_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# auto find dependencies
find_package(ament_cmake_auto REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

# proj
find_path(PROJ_INCLUDE_DIR NAMES proj.h PATHS /usr/include)
find_library(PROJ_LIBRARY NAMES proj PATHS /usr/lib/x86_64-linux-gnu)

# add_executable(gnss_cam_nav
# )
ament_auto_add_library(gnss_cam_nav
  src/follower_node.cpp
  src/path_publisher_node.cpp
)

# target_link_libraries(gnss_cam_nav_node gnssnav)
target_link_libraries(gnss_cam_nav Eigen3::Eigen)
target_link_libraries(gnss_cam_nav ${PROJ_LIBRARY})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_cmake_auto
)

target_link_libraries(${PROJECT_NAME}
  # 他の依存ライブラリ
)

ament_auto_package(
  INSTALL_TO_SHARE
)
