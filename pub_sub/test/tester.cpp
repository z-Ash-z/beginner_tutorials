#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <memory>

#include <listener.h>
#include <talker.h>

namespace integration_test {

    class Listener_Test : public Listener{
        public:
            void topic_callback(const std_msgs::msg::String &msg) override {
                RCLCPP_INFO(this->get_logger(), msg.data.c_str());
                received_msg_ = msg.data.c_str();
            }
        public:
            std::string received_msg_;
    };

    class TestingFixture : public testing::Test {
        public:
            TestingFixture()
                : node_(std::make_shared<rclcpp::Node>("basic_test"))
            {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Made the node");
            }
            void SetUp() override {
                executor_.add_node(test_talker_);
                executor_.add_node(test_listener_);
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Setup Complete");

            }
            bool TestFunc () {
                if (test_listener_->received_msg_ == "Terps Strong") {
                    return true;
                }
                else {
                    return false;
                }  
            }
            void TearDown() override {
                executor_.cancel();
                executor_.remove_node(test_talker_);
                executor_.remove_node(test_listener_);
                test_talker_.reset();
                test_listener_.reset();
                std::cout << "DONE WITH TEARDOWN" << std::endl;
            }

        protected:
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<Talker> test_talker_;
            std::shared_ptr<Listener_Test> test_listener_;
            rclcpp::executors::SingleThreadedExecutor executor_;
    };

    TEST_F(TestingFixture, BasicTest) {
        std::cout << "The actual test is happenning here!" << std::endl;
        EXPECT_TRUE(true);
    }
    
    TEST_F(TestingFixture, FucReturn) {
        std::cout << "Fuction Return check" << std::endl;

        // auto future = std::shared_future<bool>(this->TestFunc());

        executor_.spin_once();
        EXPECT_TRUE(this->TestFunc());
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