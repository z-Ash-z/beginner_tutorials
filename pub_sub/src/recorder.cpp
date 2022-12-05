/**
 * @file recorder.cpp
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief The implementation of the recorder class.
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <recorder.h>

Recorder::Recorder(const std::string &node_name) : Node(node_name) {
  // Declare parameters.
  this->declare_parameter("topic_name", "Messages");
  this->declare_parameter("bag_name", "my_bag");

  // Initializing the writer.
  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(this->get_parameter("bag_name").as_string());

  // Creating the subscriber.
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      this->get_parameter("topic_name").as_string(), 10,
      std::bind(&Recorder::recorder_callback, this, std::placeholders::_1));
}

void Recorder::recorder_callback(
    std::shared_ptr<rclcpp::SerializedMessage> msg) {
  rclcpp::Time time_stamp = this->now();
  writer_->write(msg, this->get_parameter("topic_name").as_string(),
                 "std_msgs/msg/String", time_stamp);
}
