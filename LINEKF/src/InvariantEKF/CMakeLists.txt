cmake_minimum_required(VERSION 2.8.3)
project(invariant_ekf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  can_driver
)


catkin_package(
     INCLUDE_DIRS include
)
# Add GeographicLib.
add_subdirectory(third_party/GeographicLib)

include_directories(
    include
    lib
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    third_party/GeographicLib/include
)

add_library(
    INEKF_shared 
    lib/InEKF.cpp
    lib/NoiseParams.cpp
    lib/LieGroup.cpp
    lib/RobotState.cpp
)

add_executable(
    InvariantEKF
    src/InvariantEKF.cpp
)

add_dependencies(InvariantEKF ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(InvariantEKF
    INEKF_shared
    ${catkin_LIBRARIES}
    libGeographiccc
)


