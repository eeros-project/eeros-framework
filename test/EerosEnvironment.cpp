#include <EerosEnvironment.hpp>
#include <eeros/hal/HAL.hpp>
#include <TestVariables.hpp>

using namespace eeros::test;
using namespace eeros::hal;

void EerosEnvironment::SetUp() {
	std::cout << "[----------] set-up EEROS test environment" << std::endl;
	HAL& hal = HAL::instance();
	
	if(libcomedi) {
		hal.readConfigFromFile("loadConfigComedi.json");
	}
	else if(libflink) {
		hal.readConfigFromFile("loadConfigFlink.json");
	}
	else if(libsim) {
		hal.readConfigFromFile("loadConfigSim.json");
	}
	else {
		throw eeros::EEROSException("no config loaded");
	}
}

void EerosEnvironment::TearDown() {
	std::cout << "[----------] tear-down EEROS test environment" << std::endl;
}
