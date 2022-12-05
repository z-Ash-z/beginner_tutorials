cmake_minimum_required(VERSION 3.8)
project(pub_sub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# ament_lint_auto_find_test_dependencies()

include_directories(include/pub_sub)

#--------Custom interfaces : start ----
find_package(rosidl_default_generators REQUIRED)
set(svg_files "srv/StringChange.srv")
rosidl_generate_interfaces(${PROJECT_NAME} ${svg_files})
#--------Custom interfaces : end ------

#--------Talker class : start ---------
add_executable(publisher app/publisher.cpp src/talker.cpp)
ament_target_dependencies(publisher rclcpp std_msgs geometry_msgs rclcpp tf2_ros tf2)

install(TARGETS publisher DESTINATION lib/${PROJECT_NAME})
#--------Talker class : end -----------

#--------Listener class : start -------
add_executable(subscriber app/subscriber.cpp src/listener.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

install(TARGETS subscriber DESTINATION lib/${PROJECT_NAME})
#--------Listener class : end ---------

#--------Recorder class : start -------
add_executable(recorder app/recorder.cpp src/recorder.cpp)
ament_target_dependencies(recorder rclcpp std_msgs rosbag2_cpp)

install(TARGETS recorder DESTINATION lib/${PROJECT_NAME})
#--------Recorder class : end ---------

#--------Tesing : start ---------------
# add_executable(tester test/tester.cpp src/listener.cpp src/talker.cpp)
# ament_target_dependencies(tester rclcpp std_msgs geometry_msgs rclcpp tf2_ros tf2 ament_cmake_gtest) 
set(TESTFILES test/intergation_tester.cpp)
ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs geometry_msgs rclcpp tf2_ros tf2)
#--------Tesing : end -----------------

#--------Linking interfaces : start ---
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(publisher "${cpp_typesupport_target}")
target_link_libraries(${PROJECT_NAME}_test "${cpp_typesupport_target}")
#--------Linking interfaces : end -----

#--------For Launch file : start ------
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
#--------For Launch file : end --------

ament_package()
