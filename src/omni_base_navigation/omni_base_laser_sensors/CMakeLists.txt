cmake_minimum_required(VERSION 3.8)
project(omni_base_laser_sensors)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(module/10_laser.yaml)
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()
if(pal_configuration_cmake_FOUND)
  pal_register_configs(
    CONFIGS
      params/10_laser_hokuyo_omni_base.yaml
    RESOURCE_NAME
      hokuyo_laser_cfg
  )
  pal_register_configs(
    CONFIGS
      params/10_nav2_lifecycle_manager_laser_hokuyo_omni_base.yaml
      params/10_nav2_lifecycle_manager_laser_sick-561_omni_base.yaml
      params/10_nav2_lifecycle_manager_laser_sick-571_omni_base.yaml
      params/10_nav2_lifecycle_manager_laser_sim_omni_base.yaml
    RESOURCE_NAME
      nav2_lifecycle_manager
  )
  pal_register_configs(
    CONFIGS
      params/10_pal_laser_filters_omni_base.yaml
    RESOURCE_NAME
      pal_laser_filters
  )
  pal_register_configs(
    CONFIGS
      params/10_laser_sick-561_omni_base.yaml
      params/10_laser_sick-571_omni_base.yaml
    RESOURCE_NAME
      sick_laser_cfg
  )
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_configuration_cmake package was not found. Configurations not registered.")
endif()

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
