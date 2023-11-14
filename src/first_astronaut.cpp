#include "first_astronaut.hpp"

FirstAstronaut::FirstAstronaut()
    : Node("first_astronaut") {
  publisher_ = this->create_publisher<std_msgs::msg::String>(
      "topic_flat_earth", 10);

  timer_ = this->create_wall_timer(
      500ms, std::bind(&FirstAstronaut::timer_callback, this));

  get_shot_ = this->create_service<std_srvs::srv::Trigger>(
      "/first_astronaut/get_shot",
      std::bind(&FirstAstronaut::get_shot_callback, this, std::placeholders::_1,
                                                        std::placeholders::_2));
}

void FirstAstronaut::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Wait, the earth's round? ";

  RCLCPP_INFO(this->get_logger(), message.data.c_str());

  publisher_->publish(message);
}

void FirstAstronaut::get_shot_callback(
    std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {

  if (request) {
    RCLCPP_INFO(this->get_logger(), "Gets shot and dies...");

    response->set__success(true);

    rclcpp::shutdown();
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<FirstAstronaut>());
  rclcpp::shutdown();

  return 0;
}
