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

class Talker : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Talker object.
   *
   * @param node_name The name of the node which publishes the message.
   * @param interval The interval (in seconds) that publishes the messages.
   * @param topic_name The name of the topic through which the messages have to
   * transported.
   */
  Talker(const std::string &node_name = "pub",
         std::string topic_name = "Messages", std::string message = "Terps Strong", int interval = 1);

 private:
  rclcpp::TimerBase::SharedPtr
      timer_;  //!< The pointer that points to the callback.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_;  //!< The pointer to the publisher topic.
  std_msgs::msg::String message_; //!< The message that will be broadcasted in the channel.
  

  /**
   * @brief The callback funtion that prints and publishes a message in the
   * topic.
   *
   */
  void timer_callback();
};  // Talker

#endif  // __TALKER_H__
