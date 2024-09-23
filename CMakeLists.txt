cmake_minimum_required(VERSION 3.27)
project(behavior_tree_test)

set(CMAKE_CXX_STANDARD 20)

add_custom_target(copy_target
        COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_CURRENT_SOURCE_DIR}/my_tree.xml
        ${CMAKE_BINARY_DIR}/my_tree.xml
        COMMENT "Copying 'test' library to '${CMAKE_BINARY_DIR}'")

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(behavior_tree_test main.cpp)
add_dependencies(behavior_tree_test copy_target)
ament_target_dependencies(behavior_tree_test behaviortree_cpp geometry_msgs)