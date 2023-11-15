/**
 * @file second_astronaut.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Open Source Robotics Foundation (info@openrobotics.org)
 * @brief RCLCPP Node definition that plays the second astronaut character.
 * @version 1.0
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023 Open Source Robotics Foundation, Abhishekh Reddy
 *
 */

/*
  Copyright 2023 Open Source Robotics Foundation, Abhishekh Reddy

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#include "second_astronaut.hpp"

SecondAstronaut::SecondAstronaut()
    : Node("second_astronaut") {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_flat_earth", 10, std::bind(&SecondAstronaut::listen_callback,
        this, _1));

  publisher_ = this->create_publisher<std_msgs::msg::String>(
      "topic_flat_earth", 10);

  shoot_ = this->create_client<std_srvs::srv::Trigger>(
      "/first_astronaut/get_shot");

  // Parameter that allows the user to change the meme ending
  this->declare_parameter("dramatic_end", false);
}

void SecondAstronaut::listen_callback(const std_msgs::msg::String & msg) const {
  auto reply = std_msgs::msg::String();
  reply.data = "ALWAYS HAS BEEN...";

  if (msg.data == reply.data)
    return;

  RCLCPP_WARN_STREAM(this->get_logger(), reply.data);
  publisher_->publish(reply);

  if (this->get_parameter("dramatic_end").as_bool())
    SecondAstronaut::shoot();
}

void SecondAstronaut::shoot() const {
  RCLCPP_ERROR(this->get_logger(), "Shoots...");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = shoot_->async_send_request(request);

  RCLCPP_DEBUG(this->get_logger(), "That's all for today folks!");

  rclcpp::shutdown();
}

// Main function
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<SecondAstronaut>());
  rclcpp::shutdown();

  return 0;
}
