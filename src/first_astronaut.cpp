/**
 * @file first_astronaut.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Open Source Robotics Foundation (info@openrobotics.org)
 * @brief RCLCPP Node definition that plays the first astronaut character.
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

#include "first_astronaut.hpp"

FirstAstronaut::FirstAstronaut()
    : Node("first_astronaut") {
  publisher_ = this->create_publisher<std_msgs::msg::String>(
      "chatter", 10);

  timer_ = this->create_wall_timer(
      500ms, std::bind(&FirstAstronaut::timer_callback, this));

  get_shot_ = this->create_service<std_srvs::srv::Trigger>(
      "/first_astronaut/get_shot",
      std::bind(&FirstAstronaut::get_shot_callback, this, std::placeholders::_1,
                                                        std::placeholders::_2));

  world_frame_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Parameter that allows the user to set a custom message
  this->declare_parameter("realization", "Wait, the earth's round?");
}

void FirstAstronaut::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = this->get_parameter("realization").as_string() + " ";

  RCLCPP_INFO_STREAM(this->get_logger(), message.data);

  publisher_->publish(message);
  this->make_transforms();
}

void FirstAstronaut::get_shot_callback(
    std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {

  if (request) {
    RCLCPP_FATAL(this->get_logger(), "Gets shot and dies...");

    response->set__success(true);

    rclcpp::shutdown();
  }
}

void FirstAstronaut::make_transforms() {
  geometry_msgs::msg::TransformStamped t_world;
  t_world.header.set__stamp(this->get_clock()->now());
  t_world.header.set__frame_id("world");
  t_world.set__child_frame_id("talk");

  geometry_msgs::msg::Vector3 world_translation;
  world_translation.set__x(0.3);
  world_translation.set__y(0.5);
  world_translation.set__z(0.25);
  t_world.transform.set__translation(world_translation);

  geometry_msgs::msg::Quaternion world_rotation;
  world_rotation.set__x(15 * M_PI/180);
  world_rotation.set__y(27 * M_PI/180);
  world_rotation.set__z(6 * M_PI/180);
  world_rotation.set__w(1);
  t_world.transform.set__rotation(world_rotation);

  world_frame_broadcaster_->sendTransform(t_world);
}

// Main function
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<FirstAstronaut>());
  rclcpp::shutdown();

  return 0;
}
