/**
 * @file talker.cpp
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief The implementation of the talker class.
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "talker.h"

Talker::Talker(const std::string &node_name, std::string topic_name,
               int interval)
    : Node(node_name) {
  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(interval),
                                   std::bind(&Talker::timer_callback, this));
}

void Talker::timer_callback() {
  // Storing the message
  auto message = std_msgs::msg::String();
  message.data = "Terps Strong";

  // Loggin the message in the terminal
  RCLCPP_INFO(this->get_logger(), message.data.c_str());

  // Publishing the message
  publisher_->publish(message);
}
