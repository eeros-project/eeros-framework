#include <eeros/control/ReceiveTeleopTurtleKey.hpp>
#include <chrono>

using namespace eeros::control;



ReceiveTeleopTurtleKey::ReceiveTeleopTurtleKey(ros::NodeHandle &rosNodeHandler) :
rosNodeHandler(rosNodeHandler)
{
	linearSignal.getSignal().clear();
	angularSignal.getSignal().clear();

	// ROS
	sub = rosNodeHandler.subscribe("/turtle1/cmd_vel", 1000, &ReceiveTeleopTurtleKey::rosCallback, this);
	std::cout << "n.subscribe" << std::endl;
	std::cout << "sub.getTopic()" << sub.getTopic() << std::endl;
	ROS_DEBUG("ros::Subscriber sub = n.subscribe()");
}

//ReceiveTeleopTurtleKey::ReceiveTeleopTurtleKey(std::string& name) {
//	ReceiveTeleopTurtleKey();
//}

ReceiveTeleopTurtleKey::~ReceiveTeleopTurtleKey() { }


void ReceiveTeleopTurtleKey::rosCallback(const geometry_msgs::Twist::ConstPtr& vel) {
	vel_buffered = *vel;
	auto time = eeros::System::getTimeNs();
	linearValue(0)	= vel_buffered.linear.x;
	linearValue(1)	= vel_buffered.linear.y;
	linearValue(2)	= vel_buffered.linear.z;
	angularValue(0) = vel_buffered.angular.x;
	angularValue(1) = vel_buffered.angular.y;
	angularValue(2) = vel_buffered.angular.z;
	linearSignal.getSignal().setTimestamp(time);
	linearSignal.getSignal().setValue(linearValue);
	angularSignal.getSignal().setTimestamp(time);
	angularSignal.getSignal().setValue(angularValue);

	ROS_DEBUG("ReceiveTeleopTurtleKey::rosCallback() was called");
}


void ReceiveTeleopTurtleKey::run() {
	ros::spinOnce();
}
