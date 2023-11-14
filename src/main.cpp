#include "talker.hpp"
#include "listener.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto flat_earther {std::make_shared<FlatEarther>()};
  auto normal_person {std::make_shared<NormalPerson>()};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(flat_earther);
  executor.add_node(normal_person);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
