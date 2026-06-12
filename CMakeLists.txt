cmake_minimum_required(VERSION 3.8)
project(amr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# No C++ nodes in this package — it is a description-only package.
# We keep rclcpp / std_msgs out to keep the dependency list clean.
# Add them back only if you add C++ source files under src/.

# -----------------------------------------------------------------------
# Install all resource directories into the package share folder
# -----------------------------------------------------------------------
install(
  DIRECTORY
    launch
    urdf
    meshes
    config
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
