#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <gtest/gtest.h>
#include <TestVariables.hpp>

using namespace eeros;
using namespace eeros::hal;

TEST(hal_CreateObjectsTest, availableDigInOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn");
		eeros::hal::Output<bool> &io1 = *hal.getLogicOutput("io1");
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut");
		eeros::hal::Output<bool> &ioOut4 = *hal.getLogicOutput("ioOut4");
		
		eeros::hal::Output<double> &dac1 = *hal.getRealOutput("dac1");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_CreateObjectsTest, notAvailableDigInOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &en = *hal.getLogicInput("enable56");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'enable56' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_CreateObjectsTest, emptySignalIdLogicIn){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &en = *hal.getLogicInput("");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_CreateObjectsTest, emptySignalIdLogicOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<bool> &en = *hal.getLogicOutput("");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_CreateObjectsTest, emptySignalIdRealIn){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &en = *hal.getLogicInput("");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_CreateObjectsTest, emptySignalIdRealOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &en = *hal.getRealOutput("");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_CreateObjectsTest, wrongTypeReal){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &ioIn = *hal.getRealInput("ioIn");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'ioIn' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_CreateObjectsTest, wrongTypeLogic){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &dac = *hal.getLogicInput("dac1");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'dac1' not found!"));
	}
	catch(...){
		FAIL();
	}
}