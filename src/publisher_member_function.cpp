// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file publisher_member_function.cpp
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief Code for getting custom input from user and publishing it
 * @version 0.1
 * @date 2023-11-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include <beginner_tutorials/srv/string_mod.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using sharedFuture =
    rclcpp::Client<beginner_tutorials::srv::StringMod>::SharedFuture;

/**
 * @brief MinimalPublisher class, defines the publisher, service client and the
 * associated functions
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Parameter Declaration
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Setting callback freq.";
    this->declare_parameter("freq", 1.0, param_desc);
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();
    RCLCPP_DEBUG(this->get_logger(),
                 "Parameter freq declared, setting 1.0 Hz as default");

    // Creating a subscriber for the parameter
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto parameterCallbackPtr =
        std::bind(&MinimalPublisher::param_cb, this, _1);
    param_handle_ =
        param_subscriber_->add_parameter_callback("freq", parameterCallbackPtr);

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    RCLCPP_DEBUG(this->get_logger(), "Initiated Publisher");
    auto delta = std::chrono::milliseconds(static_cast<int>((1000 / freq)));
    timer_ = this->create_wall_timer(
        delta, std::bind(&MinimalPublisher::timer_cb, this));

    client =
        this->create_client<beginner_tutorials::srv::StringMod>("modify_msg");
    RCLCPP_DEBUG(this->get_logger(), "Initiated Client");
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Interrupted service");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Unavailable service, waiting for start");
    }

    // Broadcast a tf frame info
    tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    // Translation block
    t.transform.translation.x = 10;
    t.transform.translation.y = 10;
    t.transform.translation.z = 10;

    // Rotation block
    t.transform.rotation.x = 1;
    t.transform.rotation.y = 1;
    t.transform.rotation.z = 1;
    t.transform.rotation.w = 1;

    tf_broadcaster_->sendTransform(t);
  }

 private:
  std::string Message;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::Client<beginner_tutorials::srv::StringMod>::SharedPtr client;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> param_handle_;

  /**
   * @brief Timer callback function
   */
  void timer_cb() {
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Node setup");
    auto message = std_msgs::msg::String();
    message.data = "Message published ID " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing message : '%s'",
                message.data.c_str());
    publisher_->publish(message);
    if (count_ % 10 == 0) {
      service_cb();
    }
    auto steady_clock = rclcpp::Clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), steady_clock, 10000,
                                 "Successful node execution");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  /**
   * @brief Service callback function
   * @return int
   */
  int service_cb() {
    auto request =
        std::make_shared<beginner_tutorials::srv::StringMod::Request>();
    request->a = "Message 1";
    request->b = " Message 2";
    RCLCPP_INFO(this->get_logger(), "Service called to Modify string");
    auto callbackPtr = std::bind(&MinimalPublisher::response_cb, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }

  /**
   * @brief Response callback function
   * @param future
   */
  void response_cb(sharedFuture future) {
    // Process the response
    RCLCPP_INFO(this->get_logger(), "Got String: %s", future.get()->c.c_str());
    Message = future.get()->c.c_str();
  }

  /**
   * @brief Parameter callback function
   * @param param
   */
  void param_cb(const rclcpp::Parameter &param) {
    RCLCPP_INFO(this->get_logger(),
                "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                param.get_name().c_str(), param.get_type_name().c_str(),
                param.as_double());
    RCLCPP_WARN(this->get_logger(), "Message frequency changed");

    RCLCPP_FATAL_EXPRESSION(this->get_logger(), param.as_double() == 0.0,
                            "Frequency is set to zero, zero division error");
    if (param.as_double() == 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Frequency has not been changed.");
    } else {
      auto delta = std::chrono::milliseconds(
          static_cast<int>((1000 / param.as_double())));
      timer_ = this->create_wall_timer(
          delta, std::bind(&MinimalPublisher::timer_cb, this));
    }
  }
};

/**
 * @brief Main function
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

