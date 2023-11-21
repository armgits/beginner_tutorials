/**
 * @file first_astronaut.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Open Source Robotics Foundation (info@openrobotics.org)
 * @brief RCLCPP Node declaration for the first astronaut character.
 * @version 1.0
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023 Open Source Robotics Foundation, Abhishekh Reddy
 *
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

/**
 * @brief The FirstAstronaut class inherits from the Node class in the RCLCP
 *        library. This node plays the character of the first astronaut from the
 *        "Always has been" meme.
 *
 */
class FirstAstronaut : public rclcpp::Node {
 public:
  /**
   * @brief Spawns the first astronaut character.
   *
   */
  FirstAstronaut();

 private:
  /**
   * @brief Express first astronaut's realization to the second astronaut.
   *
   */
  void timer_callback();

  /**
   * @brief Response to the trigger pulled by the second astronaut.
   *        You can expect what happens usually...
   *
   * @param request
   * @param response
   */
  void get_shot_callback(std_srvs::srv::Trigger::Request::SharedPtr request,
                         std_srvs::srv::Trigger::Response::SharedPtr response);

  // TODO: DOXYGEN BLOCK
  void make_transforms();

  /**
   * @brief Periodically calls the callback method to express realization.
   *
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Mouth for the first astronaut to talk.
   *
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  /**
   * @brief Makes the first astronaut vulnerable to trigger.
   *
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_shot_;

  // TODO: DOXYGEN BLOCK
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> world_frame_broadcaster_;
};
