cmake_minimum_required(VERSION 3.8)
# Simulation-only overlay of PAL's pal_sea_arm_description. It installs the upstream package
# content unchanged and replaces only the wrist-camera xacro, which here also declares a Gazebo
# RGB sensor. Sourcing this workspace makes $(find pal_sea_arm_description) resolve to it.
project(pal_sea_arm_description)

find_package(ament_cmake REQUIRED)

set(PAL_SEA_ARM_UNDERLAY "/opt/pal/alum/share/pal_sea_arm_description"
    CACHE PATH "Installed upstream pal_sea_arm_description share directory")
if(NOT EXISTS "${PAL_SEA_ARM_UNDERLAY}/urdf/end_effector_camera/end_effector_camera.urdf.xacro")
  message(FATAL_ERROR "upstream pal_sea_arm_description not found at ${PAL_SEA_ARM_UNDERLAY}")
endif()

foreach(dir config gazebo launch meshes module mujoco robots ros2_control urdf)
  if(EXISTS "${PAL_SEA_ARM_UNDERLAY}/${dir}")
    install(DIRECTORY "${PAL_SEA_ARM_UNDERLAY}/${dir}" DESTINATION share/${PROJECT_NAME}
            USE_SOURCE_PERMISSIONS
            PATTERN "end_effector_camera.urdf.xacro" EXCLUDE)
  endif()
endforeach()

install(FILES urdf/end_effector_camera/end_effector_camera.urdf.xacro
        DESTINATION share/${PROJECT_NAME}/urdf/end_effector_camera)

ament_package()
