cmake_minimum_required(VERSION 3.8)
project(omni_base_description)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/00_robot_state_publisher.yaml
  )
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()

#############
## Testing ##
#############

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_xacro test/test_xacro.py)
  add_launch_test(test/test_description.launch.py)
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE rviz launch meshes config robots ros2_control urdf)
