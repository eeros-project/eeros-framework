#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>

#include <eeros/core/Executor.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/FlinkDigOut.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#include "ControlSystem.hpp"

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::logger;

void initHardware() {
	HAL& hal = HAL::instance();

	std::cout << "  Creating device structure..." << std::endl;
	FlinkDevice* flink0 = new FlinkDevice("/dev/flink0");
	
	std::cout << "  Creating I/O objects..." << std::endl;
	FlinkFqd* enc0 = new FlinkFqd("q", flink0, 2, 0, 6.28318530718 / (4 * 500.0), 0, 0);
	FlinkPwm* mot0 = new FlinkPwm("mot", flink0, 1, 0);
	FlinkDigOut* en0 = new FlinkDigOut("enable", flink0, 3, 0);
	
	std::cout << "  Registering I/Os in the HAL..." << std::endl;
	hal.addInput(enc0);
	hal.addOutput(mot0);
	hal.addOutput(en0);
	
	std::cout << "  Initializing I/Os" << std::endl;
	enc0->reset();
	mot0->setFrequency(1000);
	mot0->setDutyCycle(0.5);
	en0->set(true);
}


int main() {
	std::cout << "fLink controller demo started..." << std::endl;

	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	
	std::cout << "Initializing Hardware..." << std::endl;
	initHardware();
	
	// Get control System instance
	ControlSystem& controlSys = ControlSystem::instance();

	// Start control system
	controlSys.start();
	
	for(int i = 0; i < 100; i++) {
		std::cout << controlSys.enc.getOut().getSignal().getValue() << std::endl;
		sleep(1);
	}
	
	controlSys.stop();
	
	sleep(1);
	
	std::cout << "Example finished..." << std::endl;
}
