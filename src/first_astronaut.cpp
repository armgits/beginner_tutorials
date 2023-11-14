#include "first_astronaut.hpp"

FirstAstronaut::FirstAstronaut()
    : Node("first_astronaut"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::String>(
      "topic_flat_earth", 10);

  timer_ = this->create_wall_timer(
      500ms, std::bind(&FirstAstronaut::timer_callback, this));
}

void FirstAstronaut::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Wait, the earth's round? " + std::to_string(count_++);

  RCLCPP_INFO(this->get_logger(), message.data.c_str());

  publisher_->publish(message);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<FirstAstronaut>());
  rclcpp::shutdown();

  return 0;
}
