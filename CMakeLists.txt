cmake_minimum_required(VERSION 2.8.3)
project(upc_mrn)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS move_base_msgs roscpp rospy actionlib tf visualization_msgs nav_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES upc_mrn
#  CATKIN_DEPENDS other_catkin_pkg
    CATKIN_DEPENDS move_base_msgs roscpp rospy action_lib tf visualization_msgs nav_msgs geometry_msgs
#  DEPENDS system_lib
)

add_executable(exploration_random src/exploration_random.cpp)
add_executable(exploration_biggest_frontier src/exploration_biggest_frontier.cpp)
add_executable(exploration_closest_frontier src/exploration_closest_frontier.cpp)
add_executable(exploration_project src/exploration_project2.cpp)
#add_executable(exploration_project1 src/exploration_project1.cpp)
#add_executable(exploration_project2 src/exploration_project2.cpp)

target_link_libraries(exploration_random  ${catkin_LIBRARIES})
target_link_libraries(exploration_biggest_frontier  ${catkin_LIBRARIES})
target_link_libraries(exploration_closest_frontier  ${catkin_LIBRARIES})
target_link_libraries(exploration_project  ${catkin_LIBRARIES})
#target_link_libraries(exploration_project1  ${catkin_LIBRARIES})
#target_link_libraries(exploration_project2  ${catkin_LIBRARIES})

