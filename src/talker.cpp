#include "talker.hpp"

FlatEarther::FlatEarther()
    : Node("flat_earther"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::String>(
      "topic_flat_earth", 10);

  timer_ = this->create_wall_timer(
      500ms, std::bind(&FlatEarther::timer_callback, this));
}

void FlatEarther::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Wait, the earth's round? " + std::to_string(count_++);

  RCLCPP_INFO(
    this->get_logger(),
    "Flat earther from the space says: '%s'", message.data.c_str());

  publisher_->publish(message);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<FlatEarther>());
  rclcpp::shutdown();

  return 0;
}
