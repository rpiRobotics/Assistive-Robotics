cmake_minimum_required(VERSION 3.0.2)
project(rokubi_kinova_adapter)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PACKAGE_DEPENDENCIES
  sensor_msgs
  std_msgs
  xacro
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
  ${PACKAGE_DEPENDENCIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY meshes/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
		)

install(DIRECTORY urdf/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
		)

