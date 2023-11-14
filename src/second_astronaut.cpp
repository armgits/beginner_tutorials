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
}

void SecondAstronaut::listen_callback(const std_msgs::msg::String & msg) const {
  auto reply = std_msgs::msg::String();
  reply.data = "ALWAYS HAS BEEN...";

  if (msg.data == reply.data)
    return;

  RCLCPP_INFO(this->get_logger(), reply.data.c_str());
  publisher_->publish(reply);

  SecondAstronaut::shoot();
}

void SecondAstronaut::shoot() const {
  RCLCPP_INFO(this->get_logger(), "Shoots...");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = shoot_->async_send_request(request);

  RCLCPP_INFO(this->get_logger(), "That's all for today folks...");

  rclcpp::shutdown();
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<SecondAstronaut>());
  rclcpp::shutdown();

  return 0;
}
