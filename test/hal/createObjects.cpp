#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <gtest/gtest.h>
#include <TestVariables.hpp>

using namespace eeros;
using namespace eeros::hal;

TEST(hal_CreateObjectsTest, availableDigInOut){
	HAL& hal = HAL::instance();
	try{
		if(libcomedi){
			hal.readConfigFromFile("loadConfigComedi.json");
		}
		else if(libflink){
			hal.readConfigFromFile("loadConfigFlink.json");
		}
		else{
			FAIL();
		}
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn");
		eeros::hal::Output<bool> &io1 = *hal.getLogicOutput("io1");
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut");
		eeros::hal::Output<bool> &ioOut4 = *hal.getLogicOutput("ioOut4");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}