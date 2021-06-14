cmake_minimum_required(VERSION 3.0.2)
project(rqt_rosparam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_rosparam
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  scripts/rqt_rosparam
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)