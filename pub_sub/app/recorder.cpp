/**
 * @file recorder.cpp
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief A driver script to run the ros bag recorder.
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <recorder.h>

#include <memory>

int main(int argc, char* argv[]) {
  // Initializing the rclcpp
  rclcpp::init(argc, argv);

  // Instantiating and spinning the node
  rclcpp::spin(std::make_shared<Recorder>());

  // Shutdown
  rclcpp::shutdown();

  return 0;
}
