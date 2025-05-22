^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package omni_base_2dnav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.15.0 (2025-05-07)
-------------------

2.14.0 (2025-05-07)
-------------------

2.13.0 (2025-04-03)
-------------------
* pal_configuration adoption
* Contributors: antoniobrandi

2.12.1 (2025-02-10)
-------------------

2.12.0 (2025-02-04)
-------------------
* Update omni_base_nav_bringup.launch.py
* Contributors: antoniobrandi

2.11.0 (2025-01-30)
-------------------

2.10.0 (2025-01-23)
-------------------

2.9.0 (2025-01-21)
------------------

2.8.0 (2025-01-17)
------------------

2.7.0 (2024-12-02)
------------------
* nav deps and specifics
* Contributors: antoniobrandi

2.6.1 (2024-11-21)
------------------
* start rviz with use_public_sim
* Contributors: antoniobrandi

2.6.0 (2024-11-14)
------------------
* disable costmap filters for normal nav
* register modules
* add multi-robot support
* typos
* restore laser modules
* moved camera-related variables to rgbd packag
* remove dependency from pal_nav2_bringup
* use mppi
* register nav variables and pipelines
* Contributors: antoniobrandi

2.5.0 (2024-10-23)
------------------
* using MPPI
* Contributors: andreacapodacqua

2.4.0 (2024-10-16)
------------------
* Apply 1 suggestion(s) to 1 file(s)
* Apply 1 suggestion(s) to 1 file(s)
* linters
* added unless condition to rviz
* Contributors: antoniobrandi, martinaannicelli

2.3.0 (2024-08-22)
------------------
* added docking link to variables
* Contributors: antoniobrandi

2.2.1 (2024-08-06)
------------------
* fix public sim
* Contributors: antoniobrandi

2.2.0 (2024-08-06)
------------------
* Unify quotation marks
* Unify quotation marks
* restructure launch file
* Contributors: Aina, antoniobrandi

2.1.1 (2024-07-19)
------------------
* fix laser frames
* Contributors: andreacapodacqua

2.1.0 (2024-07-17)
------------------
* reorganized remappings file
* naming convention
* added device_number_laser
* using remappings for rgbd and laser pipeline
* Contributors: andreacapodacqua

2.0.19 (2024-07-09)
-------------------
* Add warning for pal_module_cmake not found
* Contributors: Noel Jimenez

2.0.18 (2024-07-01)
-------------------
* using costmap with filters
* Contributors: antoniobrandi

2.0.17 (2024-06-25)
-------------------
* move rviz in nav launch file
* Contributors: antoniobrandi

2.0.16 (2024-06-17)
-------------------
* Fix pal_maps path
* Contributors: andreacapodacqua

2.0.15 (2024-06-03)
-------------------
* Update omni_base_2dnav/params/omni_base_nav.yaml, omni_base_2dnav/params/omni_base_nav_w_composition.yaml
* Contributors: antoniobrandi

2.0.14 (2024-06-03)
-------------------

2.0.13 (2024-05-29)
-------------------
* fix public sim
* Contributors: andreacapodacqua

2.0.12 (2024-04-29)
-------------------
* deprecate omni_base_maps
* Contributors: antoniobrandi

2.0.11 (2024-04-26)
-------------------
* Merge branch 'abr/fix/new-launch-pal' into 'humble-devel'
  Adapt to the new launch_pal
  See merge request robots/omni_base_navigation!27
* Update omni_base_remappings.yaml
* Adapt to the new launch_pal
* Contributors: antoniobrandi

2.0.10 (2024-04-23)
-------------------
* Merge branch 'feat/move-modules-to-00' into 'humble-devel'
  moved modules to 00
  See merge request robots/omni_base_navigation!25
* moved modules to 00
* Contributors: andreacapodacqua

2.0.9 (2024-04-23)
------------------
* Merge branch 'fix/variables' into 'humble-devel'
  Fix/variables
  See merge request robots/omni_base_navigation!23
* removed unused params for sim
* using variables lifecycle manager
* fix variables, laser angles
* Contributors: andreacapodacqua

2.0.8 (2024-04-17)
------------------
* Merge branch 'fix/laser-params' into 'humble-devel'
  fix laser params name
  See merge request robots/omni_base_navigation!22
* added dep
* new variables names
* using variables for pipelines
* Contributors: andreacapodacqua

2.0.7 (2024-04-10)
------------------
* Merge branch 'feat/ros2-params' into 'humble-devel'
  Feat/ros2 params
  See merge request robots/omni_base_navigation!20
* cosmetic
* launch indipendent nav loc and slam public sim
* linters
* pipelines for navigation
* fix launch private sim
* fix and change params names
* fix dep
* default nav config for omni_base
* splitted navigation and localization pipeline and modules
* added state_lattice
* rviz config
* fine tuning params
* tuning parameters ros2
* Contributors: andreacapodacqua

2.0.6 (2024-03-06)
------------------

2.0.5 (2024-03-05)
------------------
* Merge branch 'fix/laser-pipeline' into 'humble-devel'
  renamed apps and removed unuseful args
  See merge request robots/omni_base_navigation!18
* renamed apps and removed unuseful args
* Contributors: andreacapodacqua

2.0.4 (2024-02-28)
------------------
* Merge branch 'aca/experiments-module' into 'humble-devel'
  Aca/experiments module
  See merge request robots/omni_base_navigation!17
* new load of params in module
* linters and omni base radius
* remappings laser pipeline
* modules experiments
* Contributors: andreacapodacqua

2.0.3 (2024-02-02)
------------------
* Merge branch 'feat/register-components' into 'humble-devel'
  using components and parameters
  See merge request robots/omni_base_navigation!16
* linters
* using components and parameters
* Contributors: antoniobrandi

2.0.2 (2023-12-14)
------------------

2.0.1 (2023-12-11)
------------------
* Merge branch 'fix/modules-ros2' into 'humble-devel'
  moved omni modules from 00 to 10
  See merge request robots/omni_base_navigation!14
* moved omni modules from 00 to 10
* Contributors: Noel Jimenez, andreacapodacqua

2.0.0 (2023-11-23)
------------------
* Merge branch 'feat/use-module' into 'humble-devel'
  Feat/use module
  See merge request robots/omni_base_navigation!12
* removed use_sim_time slam module
* removed navigation module
* add navigation module
* update copyright
* omni_base ROS 2
* added ira_laser_tool new rviz config
* fix launch
* fix: Typo and time_offset in sick 561
* Add TODO and scan_topic to scan
* omnibase 2dnav to ROS 2:
  + colcon
  + launch.py
  + pal_navigation_cfg
* Contributors: Noel Jimenez, YueErro, andreacapodacqua

0.0.11 (2023-03-06)
-------------------

0.0.10 (2023-01-27)
-------------------
* Merge branch 'feat/map-manager' into 'ferrum-devel'
  Move to map manager
  See merge request robots/omni_base_navigation!7
* Move to map manager
* Contributors: antoniobrandi

0.0.9 (2022-08-16)
------------------

0.0.8 (2022-08-08)
------------------
* Merge branch 'update_rviz' into 'ferrum-devel'
  Add advanced navigation file + fix nav rviz + change poi
  See merge request robots/omni_base_navigation!3
* Add advanced navigation file + fix nav rviz + change poi
* Contributors: antoniobrandi, thomaspeyrucain

0.0.7 (2022-08-04)
------------------
* Merge branch 'fix-rviz-default' into 'ferrum-devel'
  Modify default rviz to show the right topics and not transparent omni_base
  See merge request robots/omni_base_navigation!2
* Modify default rviz to show the right topics and not transparent omni_base
* Contributors: antoniobrandi, thomaspeyrucain

0.0.6 (2022-07-13)
------------------

0.0.5 (2021-10-26)
------------------

0.0.4 (2021-10-06)
------------------

0.0.3 (2021-10-04)
------------------
* removing useless dependencies
* Contributors: antoniobrandi

0.0.2 (2021-09-30)
------------------

0.0.1 (2021-09-30)
------------------
* preparing release
* Omni base navigation initial commit
* Contributors: antoniobrandi
