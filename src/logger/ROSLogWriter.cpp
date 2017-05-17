#include <eeros/logger/ROSLogWriter.hpp>
#include <ros/ros.h>

using namespace std;
using namespace eeros::logger;

ROSLogWriter::ROSLogWriter()
{
	cout << "TEST: 'ROSLogWriter' stated" << endl;

	char* dummy_args[] = {NULL};
	int argc = sizeof(dummy_args)/sizeof(dummy_args[0]) - 1;
	ros::init(argc, dummy_args, "ROSLogWriter");
	ros::NodeHandle n;
	ROS_INFO("%s", "Hello World from ROSLogWriter");
	ROS_DEBUG("Hello %s", "World from ROSLogWriter");


	cout << "ROS node shuts down after 10 seconds." << endl;
	sleep(10);




	cout << "TEST: 'ROSLogWriter' finished" << endl;
}
