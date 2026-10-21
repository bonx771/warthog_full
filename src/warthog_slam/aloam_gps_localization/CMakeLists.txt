cmake_minimum_required(VERSION 3.0.2)
project(aloam_gps_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_srvs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gps_map_georef_recorder.py
  scripts/gps_amcl_initializer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/start_warthog_rtk_driver.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

