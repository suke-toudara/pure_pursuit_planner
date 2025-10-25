cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_library(pure_pursuit_planner SHARED
  src/pure_pursuit_planner_component.cpp
)

rclcpp_components_register_node(pure_pursuit_planner
  PLUGIN "pure_pursuit_planner::PurePursuitNode"
  EXECUTABLE pure_pursuit_planner_component
)


ament_auto_add_executable(pure_pursuit_planner_node
  src/pure_pursuit_planner_node.cpp 
)

target_link_libraries(pure_pursuit_planner_node
  pure_pursuit_planner
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