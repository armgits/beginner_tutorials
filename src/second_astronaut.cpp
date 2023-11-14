#include "second_astronaut.hpp"

SecondAstronaut::SecondAstronaut()
    : Node("second_astronaut") {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_flat_earth", 10, std::bind(&SecondAstronaut::listen_callback,
        this, _1));

  publisher_ = this->create_publisher<std_msgs::msg::String>(
      "topic_flat_earth", 10);
}

void SecondAstronaut::listen_callback(const std_msgs::msg::String & msg) const {
  auto reply = std_msgs::msg::String();
  reply.data = "ALWAYS HAS BEEN...";

  if (msg.data == reply.data)
    return;

  RCLCPP_INFO(this->get_logger(), "Astronaut 2: '%s'", reply.data.c_str());
  publisher_->publish(reply);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<SecondAstronaut>());
  rclcpp::shutdown();

  return 0;
}
