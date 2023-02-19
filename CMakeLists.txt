cmake_minimum_required(VERSION 2.8.3)
project(ufo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17) # C++17 to use stdc++fs
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  image_transport
)

find_package(OpenMP REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(yaml-cpp REQUIRED QUIET)
set(YAML_CPP_LIBRARIES yaml-cpp)
# find_package(GTSAM REQUIRED QUIET)

# add_message_files(
#    DIRECTORY msg
#    FILES
#    ssc_info.msg
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
#   nav_msgs
#   sensor_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL 
  # GTSAM

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime 
  message_generation
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	# ${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  # ${GTSAM_LIBRARY_DIRS}
)


###########
## Build ##
###########

# main
add_executable( ${PROJECT_NAME}_ufo src/main.cpp
  src/ssc.cpp
)

add_dependencies(${PROJECT_NAME}_ufo ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_ufo PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_ufo
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${YAML_CPP_LIBRARIES}
  stdc++fs
)

# evaluate
add_executable( ${PROJECT_NAME}_evaluate src/evaluate.cpp)
add_dependencies(${PROJECT_NAME}_evaluate ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_evaluate PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_evaluate
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${YAML_CPP_LIBRARIES}
  stdc++fs
)


