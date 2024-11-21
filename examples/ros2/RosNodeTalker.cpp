#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <eeros_msgs/msg/analog_signal.hpp>
#include <eeros_msgs/msg/digital_signal.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/detail/joy__struct.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/System.hpp>
#include <iostream>

using namespace std;

int main(int argc, char *argv[]) {
  cout << "'rosNodeTalker' started" << endl;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rosNodeTalker");
  auto topic1 = node->create_publisher<eeros_msgs::msg::AnalogSignal>("rosNodeTalker/analogSignal", 1000);
  auto topic2 = node->create_publisher<eeros_msgs::msg::DigitalSignal>("rosNodeTalker/digitalSignal", 1000);
  auto topic3 = node->create_publisher<eeros_msgs::msg::AnalogSignal>("rosNodeTalker/analogSignalVector", 1000);
  auto topic4 = node->create_publisher<eeros_msgs::msg::DigitalSignal>("rosNodeTalker/digitalSignalVector", 1000);
  auto topic5 = node->create_publisher<geometry_msgs::msg::Twist>("rosNodeTalker/twist", 1000);
  rclcpp::Rate loop_rate(1); // 5Hz

  cout << "'rosNodeTalker' initialized" << endl;
  int count = 0;
  while (rclcpp::ok()) {
    eeros_msgs::msg::AnalogSignal msg1;
    msg1.val = {(double)(count % 10) / 10};
    msg1.timestamp = eeros::control::RosTools::convertToRosTime(eeros::System::getTimeNs());

    eeros_msgs::msg::DigitalSignal msg2;
    msg2.val = {count%2==0?false:true};
    msg2.timestamp = eeros::control::RosTools::convertToRosTime(eeros::System::getTimeNs());

    eeros_msgs::msg::AnalogSignal msg3;
    msg3.val = {(double)(count % 10) / 10, (double)(count % 10) / 100};
    msg3.timestamp = eeros::control::RosTools::convertToRosTime(eeros::System::getTimeNs());

    eeros_msgs::msg::DigitalSignal msg4;
    msg4.val = {count%2==0?false:true, true, count%2==0?true:false};
    msg4.timestamp = eeros::control::RosTools::convertToRosTime(eeros::System::getTimeNs());

    geometry_msgs::msg::Twist msg5;
    msg5.linear.x = 2;
    msg5.angular.x = -0.01;

    topic1->publish(msg1);
    topic2->publish(msg2);
    topic3->publish(msg3);
    topic4->publish(msg4);
    topic5->publish(msg5);

    std::cout << count+1 << ". message sent" << std::endl;

    rclcpp::spin_some(node);

    loop_rate.sleep();
    count++;
  }

  cout << "'rosNodeTalker' finished" << endl;
  return 0;
}
