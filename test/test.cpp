/**
 * @file test.cpp
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief File to declare tests
 * @version 0.1
 * @date 2023-11-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <gtest/gtest.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Class declaration to test publisher
 *
 */
class PublisherTask : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr test_node_;
};

/**
 * @brief Adding new test case for publisher
 *
 */
TEST_F(PublisherTask, test_num_publishers) {
  test_node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub =
      test_node_->create_publisher<std_msgs::msg::String>("chatter", 10.0);

  auto number_of_publishers = test_node_->count_publishers("chatter");

  // Check number of publishers
  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}