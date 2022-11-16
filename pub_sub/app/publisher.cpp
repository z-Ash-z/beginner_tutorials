/**
 * @file publisher.cpp
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief A driver script to run the publisher node.
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <memory>

#include "../include/pub_sub/talker.h"

int main(int argc, char* argv[]) {
  // Initializing the rclcpp
  rclcpp::init(argc, argv);

  // Instantiating and spinning the node
  rclcpp::spin(std::make_shared<Talker>());

  // Shutdown
  rclcpp::shutdown();

  return 0;
}
