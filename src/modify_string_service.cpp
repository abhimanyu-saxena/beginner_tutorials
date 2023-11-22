/**
 * @file modify_string_service.cpp
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief File to run ros service
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <beginner_tutorials/srv/string_mod.hpp>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using StringMod = beginner_tutorials::srv::StringMod;

/**
 * @brief read and add two strings.
 *
 * @param request The request containing string a and string b
 * @param response The response containing the concatenated string c
 */
void add(const std::shared_ptr<StringMod::Request> request,
         std::shared_ptr<StringMod::Response> response) {
  response->c = "Added "+ request->a + " & " + request->b + " strings.";
}

/**
 * @brief process the service node.
 *
 * @param argc Number of cli args
 * @param argv Array of cli args
 * @return int Program exit code
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("modify_msg_server");

  rclcpp::Service<StringMod>::SharedPtr service =
      node->create_service<StringMod>("modify_msg", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Modifying message from service");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
