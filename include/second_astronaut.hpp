/**
 * @file second_astronaut.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Open Source Robotics Foundation (info@openrobotics.org)
 * @brief RCLCPP Node declaration for the second astronaut character.
 * @version 1.0
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023 Open Source Robotics Foundation, Abhishekh Reddy
 *
 */

#pragma once

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using std::placeholders::_1;

/**
 * @brief The SecondAstronaut class inherits from the Node class in RCLCPP library.
 *        This node plays the character of the second astronaut from the
 *        "Always has been" meme.
 *
 */
class SecondAstronaut : public rclcpp::Node {
 public:
  /**
   * @brief Spawns the second astronaut character.
   *
   */
  SecondAstronaut();

 private:
  /**
   * @brief Responds back to the first astronaut.
   *
   * @param msg
   */
  void listen_callback(const std_msgs::msg::String & msg) const;

  /**
   * @brief Pulls the trigger on the first astronaut if possible.
   *
   */
  void shoot() const;

  /**
   * @brief Ears for the second astronaut to listen to the first astronaut.
   *
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  /**
   * @brief Mouth for the second astronaut to talk.
   *
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  /**
   * @brief Trigger to pull onto the first astronaut.
   *
   */
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shoot_;
};
