/**
 * @file talker.h
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief The talker node that publishes string messages to the specified topic.
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __TALKER_H__
#define __TALKER_H__

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pub_sub/srv/string_change.hpp>

class Talker : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Talker object.
   *
   * @param node_name The name of the node which publishes the message.
   * @param service_name The name of the service that is provided.
   */
  Talker(const std::string &node_name = "pub", std::string service_name = "change_publisher_string");

 private:
  rclcpp::TimerBase::SharedPtr timer_;  //!< The pointer that points to the callback.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  //!< The pointer to the publisher.
  std_msgs::msg::String message_; //!< The message that will be broadcasted in the channel.
  rclcpp::Service<pub_sub::srv::StringChange>::SharedPtr service_; //!< The pointer to the service.
  
  /**
   * @brief The callback funtion that prints and publishes a message in the
   * topic.
   *
   */
  void timer_callback();

  /**
   * @brief The service callback that changes the string that is published.
   * 
   * @param request The request message from client.
   * @param response The response sent to client after processing the request.
   */
  void change_string(const std::shared_ptr<pub_sub::srv::StringChange::Request> request, std::shared_ptr<pub_sub::srv::StringChange::Response> response);
};  // Talker

#endif  // __TALKER_H__
