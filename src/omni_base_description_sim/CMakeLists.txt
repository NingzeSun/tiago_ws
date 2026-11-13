cmake_minimum_required(VERSION 3.8)
# Simulation-only overlay of PAL's omni_base_description. It installs the upstream package
# content unchanged and replaces only base.gazebo.xacro, which here also welds the base to the
# Gazebo world. Sourcing this workspace makes $(find omni_base_description) resolve to it. The
# upstream Python module stays on PYTHONPATH from the underlay.
project(omni_base_description)

find_package(ament_cmake REQUIRED)

set(OMNI_BASE_UNDERLAY "/opt/pal/alum/share/omni_base_description"
    CACHE PATH "Installed upstream omni_base_description share directory")
if(NOT EXISTS "${OMNI_BASE_UNDERLAY}/urdf/base/base.gazebo.xacro")
  message(FATAL_ERROR "upstream omni_base_description not found at ${OMNI_BASE_UNDERLAY}")
endif()

foreach(dir config launch meshes module robots ros2_control rviz urdf)
  if(EXISTS "${OMNI_BASE_UNDERLAY}/${dir}")
    install(DIRECTORY "${OMNI_BASE_UNDERLAY}/${dir}" DESTINATION share/${PROJECT_NAME}
            USE_SOURCE_PERMISSIONS
            PATTERN "base.gazebo.xacro" EXCLUDE)
  endif()
endforeach()

install(FILES urdf/base/base.gazebo.xacro DESTINATION share/${PROJECT_NAME}/urdf/base)

ament_package()
