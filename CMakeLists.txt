cmake_minimum_required(VERSION 2.8.3)
project(rotors_utils)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package(DJIOSDK REQUIRED)
find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp tf hector_uav_msgs geometry_msgs sensor_msgs rotors_comm nav_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf geometry_msgs sensor_msgs hector_uav_msgs nav_msgs mav_msgs rotors_comm dji_sdk
)

add_message_files(FILES
  Sim3Stamped.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS Eigen Sophus
  CATKIN_DEPENDS message_runtime  
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS}
  LIBRARIES rotors_utils 
#  CATKIN_DEPENDS other_catkin_pkg
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include 
  ${catkin_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


LIST(APPEND LINK_LIBS 
  ${catkin_LIBRARIES}
  ${Sophus_LIBRARIES}
)

add_executable(transformation
  src/transformation.cpp
  include/rotors_utils/transformation.hpp
)
add_dependencies(transformation  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transformation 
  ${catkin_LIBRARIES}
)

add_executable(trajectory_generation
  src/common.cpp
  src/trajectory_generation.cpp
  include/rotors_utils/trajectory_generation.hpp
  include/rotors_utils/common.hpp
)
add_dependencies(trajectory_generation  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  )
target_link_libraries(trajectory_generation 
  ${LINK_LIBS} 
)

add_executable(joy_pose 
  src/joy_pose.cpp
)
add_dependencies(joy_pose  ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_pose  ${catkin_LIBRARIES} 
        ${DJIOSDK_LIBRARIES})

install(TARGETS joy_pose
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(follower_pose
  include/rotors_utils/follower_pose.hpp
  src/follower_pose.cpp
)
add_dependencies(follower_pose  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(follower_pose 
  ${catkin_LIBRARIES} 
)


add_executable(buffer
  include/rotors_utils/buffer.hpp
  src/buffer.cpp
)
add_dependencies(buffer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(buffer
  ${catkin_LIBRARIES}
)

add_executable(joy_sim
  include/rotors_utils/joy_sim.h
  src/joy_sim.cpp
)
add_dependencies(joy_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_sim
  ${catkin_LIBRARIES}
)

add_executable(dji_adapter
  include/rotors_utils/dji_adapter.h
  src/dji_adapter.cpp
)
add_dependencies(dji_adapter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dji_adapter
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)