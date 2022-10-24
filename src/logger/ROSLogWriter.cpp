#include <eeros/logger/ROSLogWriter.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace eeros::logger;

ROSLogWriter::ROSLogWriter()
{
	cout << "TEST: 'ROSLogWriter' stated" << endl;

	char* dummy_args[] = {NULL};
	int dummy_argc = sizeof(dummy_args)/sizeof(dummy_args[0]) - 1;
  rclcpp::init(dummy_argc, dummy_args);
  auto node = rclcpp::Node::make_shared("ROSLogWriter");
	RCLCPP_INFO(node->get_logger(), "%s", "Hello World from ROSLogWriter");
	RCLCPP_DEBUG(node->get_logger(), "Hello %s", "World from ROSLogWriter");


	cout << "ROS node shuts down after 10 seconds." << endl;
	sleep(10);




	cout << "TEST: 'ROSLogWriter' finished" << endl;
}
