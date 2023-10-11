cmake_minimum_required(VERSION 2.8.3)
project(rqt_explain_bt)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(
  FILES 
    plugin_1.xml
    plugin_2.xml
    plugin_3.xml
    plugin_4.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
