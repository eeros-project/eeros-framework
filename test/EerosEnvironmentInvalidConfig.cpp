#include <EerosEnvironmentInvalidConfig.hpp>
#include <eeros/hal/HAL.hpp>
#include <TestVariables.hpp>

using namespace eeros::test;
using namespace eeros::hal;

void EerosEnvironmentInvalidConfig::SetUp() {
	std::cout << "[----------] set-up invalid config EEROS test environment" << std::endl;
	HAL& hal = HAL::instance();
	
	if(libcomedi) {
		try {
			hal.readConfigFromFile("invalidScaleComedi.json");
		}
		catch(eeros::EEROSException const & err){
			ASSERT_EQ(err.what(), std::string("config of scale is invalid, id: 'dac'"));
			std::cout << "[----------] invalid scale in config detected, test successful!" << std::endl;
		}
		try {
			hal.readConfigFromFile("invalidChannelComedi.json");
		}
		catch(eeros::EEROSException const & err){
			ASSERT_EQ(err.what(), std::string("no valid key found: channela0"));
			std::cout << "[----------] invalid key in config detected, test successful!" << std::endl;
		}
	}
	else if(libflink) {
		try {
			hal.readConfigFromFile("invalidScaleFlink.json");
		}
		catch(eeros::EEROSException const & err){
			EXPECT_EQ(err.what(), std::string("config of scale is invalid, id: 'dac'"));
			std::cout << "[----------] invalid config detected, test successful!" << std::endl;
		}
	}
	else {
		throw eeros::EEROSException("no library selected with command line arguments (--library foo)");
	}
}

void EerosEnvironmentInvalidConfig::TearDown() {
	std::cout << "[----------] tear-down invalid config EEROS test environment" << std::endl;
}
