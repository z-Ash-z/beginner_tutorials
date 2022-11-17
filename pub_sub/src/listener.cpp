/**
 * @file listener.cpp
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief The implementation of the listener class.
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "listener.h"

Listener::Listener(const std::string &node_name, std::string topic_name)
    : Node(node_name) {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      topic_name, 10,
      std::bind(&Listener::topic_callback, this, std::placeholders::_1));
}

void Listener::topic_callback(const std_msgs::msg::String &msg) const {
  RCLCPP_INFO(this->get_logger(), msg.data.c_str());
}
