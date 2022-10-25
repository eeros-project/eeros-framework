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
  
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger('M');
  log.info() << "ROS example started";

  HAL& hal = HAL::instance();
  hal.readConfigFromFile(&argc, argv);

  char* dummy_args[] = {NULL};
  int dummy_argc = sizeof(dummy_args)/sizeof(dummy_args[0]) - 1;
  rclcpp::init(dummy_argc, dummy_args);
  auto node = rclcpp::Node::make_shared("rosExample");
  log.trace() << "ROS node initialized.";
  
  // "builtin_interfaces::msg::Time::now()" is used to get system time
  System::useRosTime();
  
  // This part only needed, if you want to sync the executor with a gazebo simulation
  //rclcpp::CallbackGroup::SharedPtr callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  //auto subscriber = node->create_subscription<...>(topic, queueSize, std::bind(&RosExample::ros_callback, this, _1));
    
  MyControlSystem controlSystem(dt);
  MySafetyProperties safetyProperties(controlSystem);
  SafetySystem safetySystem(safetyProperties, dt);
  
  signal(SIGINT, signalHandler);	
  auto& executor = Executor::instance();
  executor.setMainTask(safetySystem);
  //executor.syncWithRosTopic(callback_group);	// sync with gazebo simulation
  executor.run();

  log.info() << "ROS example end";	
  return 0;
}
