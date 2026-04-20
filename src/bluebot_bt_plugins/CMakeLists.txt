cmake_minimum_required(VERSION 3.8)
project(bluebot_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(opennav_docking_msgs REQUIRED)

# ── Plugin shared library ─────────────────────────────────────────────────────
add_library(bluebot_bt_plugins SHARED
  src/dock_robot_by_id.cpp
)

target_include_directories(bluebot_bt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bluebot_bt_plugins
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp_v3
  pluginlib
  opennav_docking_msgs
)

install(TARGETS bluebot_bt_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY bt_trees/
  DESTINATION share/${PROJECT_NAME}/bt_trees
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_dock_robot_by_id
    test/test_main.cpp
    test/test_dock_robot_by_id.cpp
  )

  target_include_directories(test_dock_robot_by_id PRIVATE include test)

  ament_target_dependencies(test_dock_robot_by_id
    rclcpp
    rclcpp_action
    behaviortree_cpp_v3
    nav2_behavior_tree
    opennav_docking_msgs
  )

  target_link_libraries(test_dock_robot_by_id bluebot_bt_plugins)
endif()

ament_package()
