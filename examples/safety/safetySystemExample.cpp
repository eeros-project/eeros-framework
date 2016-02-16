#include <iostream>
#include <vector>
#include <initializer_list>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/hal/DummyRealInput.hpp>
#include <eeros/hal/DummyLogicInput.hpp>
#include <eeros/hal/DummyLogicOutput.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/logger/Logger.hpp>
#include <unistd.h>

#include "ExampleSafetyProperties.hpp"

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::logger;

void initHardware() {
	HAL& hal = HAL::instance();
	
	// Add system in- and outputs to the HAL
	hal.addPeripheralInput(new DummyRealInput("q0", 6.28318530718 / 2000.0, 0));
	hal.addPeripheralInput(new DummyRealInput("q1", 6.28318530718 / 2000.0, 0));
	hal.addPeripheralInput(new DummyLogicInput("emergencyStop"));
	hal.addPeripheralOutput(new DummyLogicOutput("enable0"));
	hal.addPeripheralOutput(new DummyLogicOutput("enable1"));
	hal.addPeripheralOutput(new DummyLogicOutput("brake0"));
	hal.addPeripheralOutput(new DummyLogicOutput("brake1"));
	hal.addPeripheralOutput(new DummyLogicOutput("power"));
	hal.addPeripheralOutput(new DummyLogicOutput("wd"));
}

int main() {
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	
	Logger<LogWriter> log;
	
	log.info() << "Safety System Example started...";
	
	// Get HAL instance
	HAL& hal = HAL::instance();
	
	// Initialize Hardware
	initHardware();
	
	// Create and initialize safety system
	double period = 1;
	ExampleSafetyProperties properties;
	SafetySystem safetySys(properties, period);
	
	safetySys.triggerEvent(ExampleSafetyProperties::doSwInit);

	// Create and run executor
	auto executor = eeros::Executor::instance();
	executor.setPeriod(period);
	executor.setMainTask(safetySys);
	executor.run();

	log.info() << "Example done...";
}
