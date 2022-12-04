#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <memory>

namespace integration_test {
    class TestingFixture : public testing::Test {
        public:
            TestingFixture()
                : node_(std::make_shared<rclcpp::Node>("basic_test"))
            {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Made the node");
            }
            void SetUp() override {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Setup Complete");
            }
            bool TestFunc () {
                return true;
            }
            void TearDown() override {
                std::cout << "DONE WITH TEARDOWN" << std::endl;
            }

        protected:
            rclcpp::Node::SharedPtr node_;
    };
    TEST_F(TestingFixture, BasicTest) {
        std::cout << "The actual test is happenning here!" << std::endl;
        EXPECT_TRUE(true);
    }
    TEST_F(TestingFixture, FucReturn) {
        std::cout << "Fuction Return check" << std::endl;
        EXPECT_TRUE(TestFunc());
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