cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    unitree_legged_msgs
)

catkin_package()

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

link_directories(~/unitree_ws/dependency_lib/unitree_legged_sdk/lib/cpp/${ARCH})

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ~/unitree_ws/dependency_lib/unitree_legged_sdk/include
)

link_directories(~/unitree_ws/dependency_lib/faceLightSDK_Nano/lib)

set(EXTRA_LIBS2 -pthread libfaceLight_SDK_amd64.so)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ~/unitree_ws/dependency_lib/faceLightSDK_Nano/include
)


add_executable(example_walk src/exe/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(example_position src/exe/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(example_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(state_sub src/exe/state_sub.cpp)
target_link_libraries(state_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(state_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_udp src/exe/ros_udp.cpp)
target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(control_via_keyboard src/exe/control_via_keyboard.cpp)
target_link_libraries(control_via_keyboard ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(control_via_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(twist_sub src/exe/twist_sub.cpp)
target_link_libraries(twist_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(twist_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## mode
add_executable(stand_down src/exe/mode_5_standdown.cpp)
target_link_libraries(stand_down ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(stand_down ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(stand_up src/exe/mode_6_standup.cpp)
target_link_libraries(stand_up ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(stand_up ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(damping src/exe/mode_7_damping.cpp)
target_link_libraries(damping ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(damping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(recovery_stand src/exe/mode_8_recovery_stand.cpp)
target_link_libraries(recovery_stand ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(recovery_stand ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## skip mode 9 back_flip
add_executable(jump_and_yaw90 src/exe/mode_10_jump_and_yaw90.cpp)
target_link_libraries(jump_and_yaw90 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(jump_and_yaw90 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(praying src/exe/mode_11_praying.cpp)
target_link_libraries(praying ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(praying ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(dance1 src/exe/mode_12_dance1.cpp)
target_link_libraries(dance1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dance1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(dance2 src/exe/mode_13_dance2.cpp)
target_link_libraries(dance2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dance2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## function
add_executable(turn_off_pose src/exe/turn_off_pose.cpp)
target_link_libraries(turn_off_pose ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(turn_off_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(led src/exe/led_control.cpp)
target_link_libraries(led ${EXTRA_LIBS2} ${catkin_LIBRARIES})
add_dependencies(led ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(set_led src/exe/set_led_color.cpp)
target_link_libraries(set_led ${EXTRA_LIBS2} ${catkin_LIBRARIES})
add_dependencies(set_led ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
