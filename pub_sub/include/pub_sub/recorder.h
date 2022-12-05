/**
 * @file recorder.h
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief The recorder node that records the ros bag to the specified topic.
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef PUB_SUB_INCLUDE_PUB_SUB_RECORDER_H_
#define PUB_SUB_INCLUDE_PUB_SUB_RECORDER_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class Recorder : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Recorder object
   *
   * @param node_name The name of the node that records the ros bags.
   */
  Recorder(const std::string &node_name = "ros_bag_recorder");

 protected:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  //!< The pointer to the subscriber.
  std::unique_ptr<rosbag2_cpp::Writer>
      writer_;  //!< The pointer to the ros bag writer.

  /**
   * @brief The recorder callback that records the messages in the topic.
   *
   * @param msg The messages that are being recorded.
   */
  virtual void recorder_callback(
      std::shared_ptr<rclcpp::SerializedMessage> msg);
};  // Recorder

#endif  // PUB_SUB_INCLUDE_PUB_SUB_RECORDER_H_
