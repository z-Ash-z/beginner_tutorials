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

#ifndef PUB_SUB_INCLUDE_PUB_SUB_TALKER_H_
#define PUB_SUB_INCLUDE_PUB_SUB_TALKER_H_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <pub_sub/srv/string_change.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class Talker : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Talker object.
   *
   * @param node_name The name of the node which publishes the message.
   * @param service_name The name of the service that is provided.
   */
  Talker(const std::string &node_name = "pub",
         std::string service_name = "change_publisher_string");

 private:
  rclcpp::TimerBase::SharedPtr
      timer_publisher_;  //!< The pointer to the publisher callback.
  rclcpp::TimerBase::SharedPtr 
      timer_frame_; //!< The pointer to the frame callback.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_;  //!< The pointer to the publisher.
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_; //!< The pointer to the frame broadcaster.
  std_msgs::msg::String
      message_;  //!< The message that will be broadcasted in the channel.
  rclcpp::Service<pub_sub::srv::StringChange>::SharedPtr
      service_;  //!< The pointer to the service.

  /**
   * @brief The callback funtion that prints and publishes a message in the
   * topic.
   *
   */
  void timer_callback();

  /**
   * @brief The callback fuction that broadcasts the frame of this node.
   * 
   */
  void broadcast_timer_callback();

  /**
   * @brief The service callback that changes the string that is published.
   *
   * @param request The request message from client.
   * @param response The response sent to client after processing the request.
   */
  void change_string(
      const std::shared_ptr<pub_sub::srv::StringChange::Request> request,
      std::shared_ptr<pub_sub::srv::StringChange::Response> response);
};  // Talker

#endif  // PUB_SUB_INCLUDE_PUB_SUB_TALKER_H_
