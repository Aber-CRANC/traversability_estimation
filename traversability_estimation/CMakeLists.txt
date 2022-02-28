cmake_minimum_required(VERSION 3.10)
project(traversability_estimation)

# Use C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# include_directories(
#   include
#   ${PCL_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
#   # ${KINDR_ROS_INCLUDE_DIRS}
# )

# link_directories(
#   ${PCL_LIBRARY_DIRS}
#   ${EIGEN3_LIBRARY_DIRS}
# )

# add_definitions(${PCL_DEFINITIONS})

# Our custom pcl type with precompiled template instantiations.
# ament_auto_add_library(${PROJECT_NAME}_pcl_types
#   src/PointXYZRGBConfidenceRatio.cpp
# )

ament_auto_add_library(${PROJECT_NAME}
  src/TraversabilityMap.cpp
)

# target_link_libraries(${PROJECT_NAME}
#  ${PCL_LIBRARIES})
#  ${PROJECT_NAME}_pcl_types

# ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)


# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME})

# ament_auto_add_executable(${PROJECT_NAME}_prio_node src/${PROJECT_NAME}_node_prio_threads.cpp)
# target_link_libraries(${PROJECT_NAME}_prio_node
#   ${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # ignore copyright checks
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright
  )

  # run cpplint without copyright filter
  find_package(ament_cmake_cpplint)
  ament_cpplint(
    FILTERS -legal/copyright
  )

  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # add_subdirectory(test)
endif()

install(
  DIRECTORY
    include/${PROJECT_NAME}
  DESTINATION
    include/${PROJECT_NAME}
)

ament_auto_package()



# cmake_minimum_required(VERSION 3.1)
# project(traversability_estimation)

# set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

# # Set the build type.  Options are:
# #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
# #  Debug          : w/ debug symbols, w/o optimization
# #  Release        : w/o debug symbols, w/ optimization
# #  RelWithDebInfo : w/ debug symbols, w/ optimization
# #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# #set(CMAKE_BUILD_TYPE RelWithDebInfo)

# ## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   cmake_modules
#   grid_map_ros
#   grid_map_core
#   grid_map_msgs
#   grid_map_filters
#   roscpp
#   tf
#   tf_conversions
#   traversability_estimation_filters
#   traversability_interfaces
#   std_msgs
#   geometry_msgs
#   sensor_msgs
#   param_io
#   xmlrpcpp
# )

# ## System dependencies are found with CMake's conventions
# find_package(Eigen3 REQUIRED)
# find_package(kindr QUIET)
# find_package(PkgConfig REQUIRED)
# if(NOT kindr_FOUND)
#   # Attempt to find package-based kindr
#   pkg_check_modules(kindr kindr REQUIRED)
# endif()


# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if you package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   INCLUDE_DIRS
#     include
#   LIBRARIES ${PROJECT_NAME} ${Eigen_INCLUDE_DIRS}
#   CATKIN_DEPENDS
#     grid_map_ros
#     grid_map_core
#     grid_map_msgs
#     grid_map_filters
#     roscpp
#     tf
#     tf_conversions
#     traversability_estimation_filters
#     traversability_interfaces
#     std_msgs
#     geometry_msgs
#     sensor_msgs
#     param_io
#     xmlrpcpp
# #  DEPENDS Eigen
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Eigen_INCLUDE_DIRS}
#   ${kindr_INCLUDE_DIRS}
# )

# ## Declare a cpp library
# add_library(
#   ${PROJECT_NAME}
#   src/TraversabilityMap.cpp
# )

# target_link_libraries(
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

# ## Declare a cpp executable
# add_executable(
#   ${PROJECT_NAME}_node
#   src/traversability_estimation_node.cpp
#   src/TraversabilityEstimation.cpp
# )

# ## Specify libraries to link a library or executable target against
# target_link_libraries(
#   ${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
# )

# #############
# ## Install ##
# #############

# # all install targets should use catkin DESTINATION variables
# # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# ## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY config launch maps
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
