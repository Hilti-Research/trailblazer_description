cmake_minimum_required(VERSION 2.8.3)
project(trailblazer_description)


find_package(catkin REQUIRED COMPONENTS roslaunch)


catkin_package()

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/multical_to_urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
