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

Talker::Talker(const std::string &node_name, std::string topic_name, std::string service_name, std::string message, int interval)
    : Node(node_name) {
  // Setting the message that will be published.
  message_.data = message;

  // Adding information in log file.
  if (rcutils_logging_set_logger_level(this->get_name(), RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) == RCUTILS_RET_OK) RCLCPP_DEBUG(this->get_logger(), "Started with DEBUG");
  else RCLCPP_INFO(this->get_logger(), "Started without DEBUG");
  RCLCPP_DEBUG(this->get_logger(), "[%s] started publishing %s to %s at %i msec", node_name.c_str(), message.c_str(), topic_name.c_str(), interval);

  // Creating a publisher.
  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
  
  // Creating a service.
  service_ = this->create_service<pub_sub::srv::StringChange>(service_name, std::bind(&Talker::change_string, this, std::placeholders::_1, std::placeholders::_2));

  // Starting the publisher.
  timer_ = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&Talker::timer_callback, this));

  RCLCPP_WARN(this->get_logger(), "This prints at start?"); // Answer to this question is yess

  // Using ERROR level logging, not necessary for this usecase but just for this assignment.
  if (interval < 500) RCLCPP_ERROR(this->get_logger(), "The speed of publish is too high!");
}

void Talker::timer_callback() {
  // Logging the message in the terminal.
  RCLCPP_INFO(this->get_logger(), message_.data.c_str());

  // Publishing the message.
  publisher_->publish(message_);
}

void Talker::change_string(const std::shared_ptr<pub_sub::srv::StringChange::Request> request, std::shared_ptr<pub_sub::srv::StringChange::Response> response)
{
  message_.data = request->new_string;
  RCLCPP_WARN(this->get_logger(), "String changed to : %s", request->new_string.c_str());
  response->change_status = "OK";
}