#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/BatteryState.h>


using namespace std;

int main(int argc, char *argv[])
{
	cout << "'rosNodeTalker' started" << endl;


	ros::init(argc, argv, "rosNodeTalker");
	ros::NodeHandle n;

	ros::Publisher chatter_topic1 = n.advertise<std_msgs::Float64>("rosNodeTalker/val", 1000);
	ros::Publisher chatter_topic2 = n.advertise<std_msgs::Float64MultiArray>("rosNodeTalker/vector", 1000);
	ros::Publisher chatter_topic3 = n.advertise<sensor_msgs::Joy>("rosNodeTalker/TestTopic3", 1000);
	ros::Publisher chatter_topic4 = n.advertise<sensor_msgs::LaserScan>("rosNodeTalker/TestTopic4", 1000);
	ros::Publisher chatter_topic5 = n.advertise<sensor_msgs::BatteryState>("rosNodeTalker/batteryState", 1000);
	ros::Rate loop_rate(5);	// 5Hz

	cout << "'rosNodeTalker' initialized" << endl;
	int count = 0;
	while (ros::ok())
	{
		std_msgs::Float64 msg1;
		std_msgs::Float64MultiArray msg2;
		sensor_msgs::Joy msg3;
		sensor_msgs::LaserScan msg4;
		sensor_msgs::BatteryState msg5;

		msg1.data = static_cast<double>( static_cast<int>(count) % 17 );

		msg2.data = {static_cast<double>(static_cast<int>(count) % 37), static_cast<double>( static_cast<int>(count) % 73 )};

		msg3.header.stamp = ros::Time::now();
		sensor_msgs::Joy::_axes_type axes {(float)(count/10), (float)((count+1)/10), (float)((count+2)/10)};
		msg3.axes = axes;
		sensor_msgs::Joy::_buttons_type buttons {count, count+1, count+2, count+3, count+4};
		msg3.buttons = buttons;

		msg4.header.stamp = ros::Time::now();
		msg4.angle_min = (count+10)/10;
		msg4.angle_max = (count+11)/10;
		msg4.angle_increment = (count+12)/10;
		msg4.scan_time = (count+13)/10;
		msg4.range_min = (count+14)/10;
		msg4.range_max = (count+15)/10;
		msg4.ranges = axes;
		msg4.intensities = axes;

		msg5.header.stamp = ros::Time::now();
		msg5.present = static_cast<bool>( static_cast<int>(count)%3 );



		chatter_topic1.publish(msg1);
		chatter_topic2.publish(msg2);
		chatter_topic3.publish(msg3);
		chatter_topic4.publish(msg4);
		chatter_topic5.publish(msg5);
		
		std::cout << count+1 << ". message sent" << std::endl;

		ros::spinOnce();

		loop_rate.sleep();
		count++;
	}

	cout << "'rosNodeTalker' finished" << endl;
	return 0;
}
