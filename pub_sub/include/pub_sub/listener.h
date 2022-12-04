/**
 * @file listener.h
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief The listener node that listens to the string messages in the specified
 * topic.
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PUB_SUB_INCLUDE_PUB_SUB_LISTENER_H_
#define PUB_SUB_INCLUDE_PUB_SUB_LISTENER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Listener : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Listener object.
   *
   * @param node_name The name of the node which publishes the message.
   * @param topic_name The name of the topic through which the messages have to
   * transported.
   *
   */
  Listener(const std::string &node_name = "sub",
           std::string topic_name = "Messages");

 private:
  /**
   * @brief A callback to read the message in the topic.
   *
   * @param msg The message that is read from the topic.
   */
  virtual void topic_callback(const std_msgs::msg::String &msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  //!< The pointer that subscribes to the topic.
};                    // Listener

#endif  // PUB_SUB_INCLUDE_PUB_SUB_LISTENER_H_
