cmake_minimum_required(VERSION 3.8)
project(pub_sub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#--------Talker class : start-----
include_directories(include/pub_sub)
add_executable(publisher app/publisher.cpp src/talker.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

install(TARGETS
publisher
DESTINATION lib/${PROJECT_NAME})
#--------Talker class : end-------

#--------Listener class : start --------
# include_directories(include/pub_sub)
add_executable(subscriber app/subscriber.cpp src/listener.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

install(TARGETS
subscriber
DESTINATION lib/${PROJECT_NAME})
#--------Listener class : end ----------

ament_package()
