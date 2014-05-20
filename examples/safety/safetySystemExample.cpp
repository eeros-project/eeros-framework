#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/core/EEROSException.hpp>
#include <eeros/hal/ComediDigIn.hpp>
#include <eeros/hal/ComediDigOut.hpp>
#include <eeros/hal/ComediFqd.hpp>

#include "ExampleSafetyProperties.hpp"

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

void initHardware() {
	HAL& hal = HAL::instance();
	
	// Define system in- and outputs
	ComediDevice* comedi0 = new ComediDevice("/dev/comedi0");
	
	// Add system in- and outputs to the HAL
	hal.addPeripheralInput(new ComediFqd("q0", comedi0, 11, 8, 10, 9, 6.28318530718 / 2000.0, 0, 0));
	hal.addPeripheralInput(new ComediFqd("q1", comedi0, 11, 3, 11, 4, 6.28318530718 / 2000.0, 0, 0));
	hal.addPeripheralInput(new ComediDigIn("emergencyStop", comedi0, 2, 0));
	hal.addPeripheralOutput(new ComediDigOut("enable0", comedi0, 2, 8));
	hal.addPeripheralOutput(new ComediDigOut("enable1", comedi0, 2, 9));
	hal.addPeripheralOutput(new ComediDigOut("brake0", comedi0, 2, 10));
	hal.addPeripheralOutput(new ComediDigOut("brake1", comedi0, 2, 11));
	hal.addPeripheralOutput(new ComediDigOut("power", comedi0, 2, 12));
	hal.addPeripheralOutput(new ComediDigOut("wd", comedi0, 2, 13));
}

int main() {
	std::cout << "Safety System Example started..." << std::endl;
	
	// Get HAL instance
	HAL& hal = HAL::instance();
	
	// Initialize Hardware
	initHardware();
	
	// Create and initialize safety system
	ExampleSafetyProperties properties;
	SafetySystem safetySys(properties, 1);
	
	sleep(20);
	
	std::cout << "Stopping safety system..." << std::endl;
	safetySys.stop();
	
	std::cout << "Example done..." << std::endl;
}
