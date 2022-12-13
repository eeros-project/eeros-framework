
#include "eerosRosTest.hpp"

MinimalPublisher::MinimalPublisher(rclcpp::Node::SharedPtr node)
    : node(node),
      doublePublisher(node, "/test/val"),
      doubleSignal(1.0) {
  doublePublisher.getIn().connect(doubleSignal.getOut());
  timer = node->create_wall_timer(1s, std::bind(&MinimalPublisher::do_something, this));
}

void MinimalPublisher::do_something() {
  Logger log = Logger::getLogger();
  log.info() << "Run the input block and publisher...";

  doubleSignal.setValue(doubleSignal.getValue() + 1.1);

  doubleSignal.run();
  doublePublisher.run();
}

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.info() << "Started";

  auto node = rosTools::initNode("EEROSNode");
  if (node != nullptr) {
    log.info() << "ROS node initialized: " << node->get_name();
  }

  auto p = MinimalPublisher(node);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
