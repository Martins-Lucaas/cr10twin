cmake_minimum_required(VERSION 3.8)
project(gazebo_mimic_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# gazebo_dev exporta GAZEBO_INCLUDE_DIRS / GAZEBO_LIBRARIES / GAZEBO_LIBRARY_DIRS
link_directories(${GAZEBO_LIBRARY_DIRS})

# Nome do .so casado com o filename= usado no URDF:
#   filename="libgazebo_mimic_joint_plugin.so"
add_library(gazebo_mimic_joint_plugin SHARED src/mimic_joint_plugin.cpp)
target_include_directories(gazebo_mimic_joint_plugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(gazebo_mimic_joint_plugin
  ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_mimic_joint_plugin gazebo_dev gazebo_ros)

install(TARGETS gazebo_mimic_joint_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# <prefix>/lib entra no GAZEBO_PLUGIN_PATH ao dar source no workspace.
# (Ao subir a sim pelo gazebo_ros, o gazebo_ros_paths_plugin ja varre o
#  AMENT_PREFIX_PATH; o hook cobre `gazebo` rodado na mao.)
ament_environment_hooks("env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
