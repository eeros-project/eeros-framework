#pragma once

#include <eeros/control/Constant.hpp>
#include <eeros/control/ros/RosPublisherDouble.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/control/ros/EerosRosTools.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::logger;
using namespace std::chrono_literals;

class MinimalPublisher
{
public:
  MinimalPublisher(rclcpp::Node::SharedPtr node);

  void do_something();

private:
  rclcpp::Node::SharedPtr node;
  RosPublisherDouble doublePublisher;
  Constant<double> doubleSignal;

  rclcpp::TimerBase::SharedPtr timer;
};
