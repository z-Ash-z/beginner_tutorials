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

Talker::Talker(const std::string &node_name, std::string service_name)
    : Node(node_name) {
  // Setting the message that will be published.
  // message_.data = message;

  // Declare parameters.
  this->declare_parameter("topic_name", "Messages");
  this->declare_parameter("publish_message", "Terps Strong");
  this->declare_parameter("publish_interval", 1000);

  message_.data = this->get_parameter("publish_message").as_string();

  // Adding information in log file.
  if (rcutils_logging_set_logger_level(this->get_name(), RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) == RCUTILS_RET_OK) RCLCPP_DEBUG(this->get_logger(), "Started with DEBUG");
  else RCLCPP_INFO(this->get_logger(), "Started without DEBUG");
  RCLCPP_DEBUG(this->get_logger(), "[%s] started publishing '%s' to '/%s' at %imsec", node_name.c_str(), this->get_parameter("publish_message").as_string().c_str(), this->get_parameter("topic_name").as_string().c_str(), (int)this->get_parameter("publish_interval").as_int());

  // Creating a publisher.
  publisher_ = this->create_publisher<std_msgs::msg::String>(this->get_parameter("topic_name").as_string(), 10);
  
  // Creating a service.
  service_ = this->create_service<pub_sub::srv::StringChange>(service_name, std::bind(&Talker::change_string, this, std::placeholders::_1, std::placeholders::_2));

  // Starting the publisher.
  timer_ = this->create_wall_timer(std::chrono::milliseconds(this->get_parameter("publish_interval").as_int()), std::bind(&Talker::timer_callback, this));

  // Using FATAL level logging, not necessary for this usecase but just for this assignment.
  if (this->get_parameter("publish_interval").as_int() < 500)
  {
    RCLCPP_FATAL(this->get_logger(), "The speed of publish is extremely high! Quiting.");
    exit(2);
  }

  // Using ERROR level logging, not necessary for this usecase but just for this assignment.
  if (this->get_parameter("publish_interval").as_int() < 1000) RCLCPP_ERROR(this->get_logger(), "The speed of publish is too high!");
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
  RCLCPP_DEBUG(this->get_logger(), "Now publishing '%s'", message_.data.c_str());
  response->change_status = "OK";
}
