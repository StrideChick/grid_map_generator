cmake_minimum_required(VERSION 3.8)
project(grid_map_generator)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(grid_map_generator SHARED
 src/grid_map_generator.cpp)

rclcpp_components_register_nodes(grid_map_generator
  "grid_map_generator::GridMapGenerator"
)

ament_auto_add_executable(grid_map_generator_node
    NO_TARGET_LINK_LIBRARIES
    src/grid_map_generator_node.cpp
)

target_link_libraries(grid_map_generator_node
  grid_map_generator
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES DESTINATION share/${PROJECT_NAME})
ament_auto_package()
