#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/detail/joy__struct.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <eeros/control/ros/EerosRosTools.hpp>
#include <eeros/core/System.hpp>


using namespace std;

int main(int argc, char *argv[])
{
	cout << "'rosNodeTalker' started" << endl;

	rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("rosNodeTalker");

	auto chatter_topic1 = n->create_publisher<std_msgs::msg::Float64>("rosNodeTalker/val", 1000);
	auto chatter_topic2 = n->create_publisher<std_msgs::msg::Float64MultiArray>("rosNodeTalker/vector", 1000);
	auto chatter_topic3 = n->create_publisher<sensor_msgs::msg::Joy>("rosNodeTalker/TestTopic3", 1000);
	auto chatter_topic4 = n->create_publisher<sensor_msgs::msg::LaserScan>("rosNodeTalker/TestTopic4", 1000);
	auto chatter_topic5 = n->create_publisher<sensor_msgs::msg::BatteryState>("rosNodeTalker/state", 1000);
	rclcpp::Rate loop_rate(5); // 5Hz

	cout << "'rosNodeTalker' initialized" << endl;
	int count = 0;
	while (rclcpp::ok())
	{
		std_msgs::msg::Float64 msg1;
		std_msgs::msg::Float64MultiArray msg2;
		sensor_msgs::msg::Joy msg3;
		sensor_msgs::msg::LaserScan msg4;
		sensor_msgs::msg::BatteryState msg5;

		msg1.data = static_cast<double>( static_cast<int>(count) % 17 );

		msg2.data = {static_cast<double>(static_cast<int>(count) % 37), static_cast<double>( static_cast<int>(count) % 73 )};

		msg3.header.set__stamp(eeros::control::rosTools::convertToRosTime(eeros::System::getTimeNs()));
		sensor_msgs::msg::Joy::_axes_type axes {(float)(count/10), (float)((count+1)/10), (float)((count+2)/10)};
		msg3.axes = axes;
		sensor_msgs::msg::Joy::_buttons_type buttons {count, count+1, count+2, count+3, count+4};
		msg3.buttons = buttons;

		msg4.header.set__stamp(eeros::control::rosTools::convertToRosTime(eeros::System::getTimeNs()));
		msg4.angle_min = (count+10)/10;
		msg4.angle_max = (count+11)/10;
		msg4.angle_increment = (count+12)/10;
		msg4.scan_time = (count+13)/10;
		msg4.range_min = (count+14)/10;
		msg4.range_max = (count+15)/10;
		msg4.ranges = axes;
		msg4.intensities = axes;

		msg5.header.set__stamp(eeros::control::rosTools::convertToRosTime(eeros::System::getTimeNs()));
		msg5.present = static_cast<bool>( static_cast<int>(count)%3 );

		chatter_topic1->publish(msg1);
		chatter_topic2->publish(msg2);
		chatter_topic3->publish(msg3);
		chatter_topic4->publish(msg4);
		chatter_topic5->publish(msg5);
		
		std::cout << count+1 << ". message sent" << std::endl;

		rclcpp::spin_some(n);
		loop_rate.sleep();
		count++;
	}

	cout << "'rosNodeTalker' finished" << endl;
	return 0;
}
