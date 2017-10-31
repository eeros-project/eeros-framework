#include "RosExample.hpp"

#include <iostream>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;


void signalHandler(int signum) {
	Executor::stop();
}

// This callback function is only needed, if you want to sync the executor with a gazebo simulation
// void callback(const sensor_msgs::JointState::Type){
// 	std::cout << "callback" << std::endl;
// };

int main(int argc, char **argv) {	
	double dt = 0.2;
	
	// Create and initialize logger
	// ////////////////////////////////////////////////////////////////////////
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	w.show();
 
	log.info() << "HAL ROS tester started";

	// HAL
	// ////////////////////////////////////////////////////////////////////////
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);

	// ROS
	// ////////////////////////////////////////////////////////////////////////
	char* dummy_args[] = {NULL};
	int dummy_argc = sizeof(dummy_args)/sizeof(dummy_args[0]) - 1;
	ros::init(dummy_argc, dummy_args, "rosExample");
	ros::NodeHandle rosNodeHandler;
	log.trace() << "ROS node initialized.";
	
	eeros::System::useRosTime();	// "ros::Time::now()" is used to get system time
	

	// This part only needed, if you want to sync the executor with a gazebo simulation
// 	ros::NodeHandle syncNodeHandler;
// 	ros::CallbackQueue syncCallbackQueue;
// 	syncNodeHandler.setCallbackQueue(&syncCallbackQueue);
// 	auto subscriberSync = syncNodeHandler.subscribe("motor_sim/joint_states", 1, &callback);
		
	// Control System
	// ////////////////////////////////////////////////////////////////////////
	MyControlSystem controlSystem(dt, rosNodeHandler);
	
	// Safety System
	// ////////////////////////////////////////////////////////////////////////
	MySafetyProperties safetyProperties;
	eeros::safety::SafetySystem safetySystem(safetyProperties, dt);
	
	// Executor
	// ////////////////////////////////////////////////////////////////////////
	signal(SIGINT, signalHandler);	
	auto &executor = Executor::instance();
	executor.setMainTask(safetySystem);
// 	executor.syncWithRosTopic(&syncCallbackQueue);	// sync with gazebo simulation
	executor.run();
	
	return 0;
}