cmake_minimum_required(VERSION 2.8.3)
project(simple_ekf)

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
#  LIBRARIES SimpleEKF
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
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
    EKF_shared 
    lib/EKF.cpp
    lib/CarState.cpp
    lib/LieGroup.cpp
)

add_executable(
    SimpleEKF
    src/SimpleEKF.cpp
)

add_dependencies(SimpleEKF ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(SimpleEKF
    EKF_shared
    ${catkin_LIBRARIES}
    libGeographiccc
)


