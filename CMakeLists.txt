cmake_minimum_required(VERSION 2.8.3)
project(shimizu_frenet_planner)

add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  # tf2
  autoware_msgs
  vector_map
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  tf2_ros
  # tf2
  autoware_msgs
  vector_map
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)

add_library(lattice_library src/model/polynomial_spline_model.cpp src/vehicle_info/vehicle_info.cpp
                            src/planner/trajectory_generator.cpp src/planner/lattice_planner.cpp
                            src/vector_map/vectormap_ros.cpp)
target_link_libraries(lattice_library ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(lattice src/lattice_planner_node.cpp)
target_link_libraries(lattice lattice_library ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(lattice
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
        lattice lattice_library 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
