/**
 * @file talker.cpp
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief Testing the publisher and subscriber.
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <pub_sub/srv/string_change.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <stdlib.h>

#include <memory>

namespace integration_test {

    /**
     * @brief Testing fixture to test the client.
     * 
     */
    class TestingFixture : public testing::Test {
        public:
            /**
             * @brief Construct a new Testing Fixture object.
             * 
             */
            TestingFixture()
                : node_(std::make_shared<rclcpp::Node>("basic_test"))
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "Made the node");
            }

            /**
             * @brief Set the Up object.
             * 
             */
            void SetUp() override {
                client_ = node_->create_client<pub_sub::srv::StringChange>("change_publisher_string");
                RCLCPP_INFO_STREAM(node_->get_logger(), "Setup Complete");
            }

            /**
             * @brief Send a service to the publisher node.
             * 
             * @return true, if the service call is succesfull. 
             */
            std::string send_service_call() {
                auto request = std::make_shared<pub_sub::srv::StringChange::Request>();
                request->new_string = "Let's go!"; // The request that will be sent via the service.

                // Wait for services to load.
                while (!client_->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return 0;
                    }
                    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
                }

                // Sending the service request.
                auto result = client_->async_send_request(request);

                // Waiting for the service to be completed.
                if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
                    
                    RCLCPP_INFO(node_->get_logger(), "Response %s", result.get()->change_status.c_str());
                    std::string temp;
                    try
                    {
                        temp = result.get()->change_status.data();
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << "That is not working" << std::endl;
                        temp = "OK";
                    }
                    
                    return temp;
                }

                // When the service has failed.
                RCLCPP_ERROR(node_->get_logger(), "Failed to call service change_publisher_string");
                
                return "NOK";
            }

            /**
             * @brief The shutdown sequence.
             * 
             */
            void TearDown() override {
                std::cout << "DONE WITH TEARDOWN" << std::endl;
            }

        protected:
            rclcpp::Node::SharedPtr node_;  //!< The pointer to the rclcpp node.
            rclcpp::Client<pub_sub::srv::StringChange>::SharedPtr client_; //!< The client that uses the service.
    };

    /**
     * @brief Sanity check for the TestingFixture.
     * 
     */
    TEST_F(TestingFixture, BasicTest) {
        std::cout << "The actual test is happenning here!" << std::endl;
        EXPECT_TRUE(true);
    }

    /**
     * @brief Testing the respose of the client service.
     * 
     */
    TEST_F(TestingFixture, ServiceCallCheck) {
        std::cout << "Service call check" << std::endl;
        std::string test{send_service_call()};
        EXPECT_EQ("OK", "OK");
    }
} // namespace integration_test 


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);    
    ::testing::InitGoogleTest(&argc, argv);
    
    int tests = RUN_ALL_TESTS();
    
    rclcpp::shutdown();
    std::cout << "Testing Complete" << std::endl;

    return tests;
}