#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <gtest/gtest.h>
#include <TestVariables.hpp>
#include <eeros/hal/ScalableOutput.hpp>

using namespace eeros;
using namespace eeros::hal;

TEST(hal_LoadConfigFileTest, noFile){
	HAL& hal = HAL::instance();
	
	try{
		hal.readConfigFromFile("");
	}
	catch(eeros::EEROSException const & err){
	      EXPECT_EQ(err.what(), std::string("cannot open file : No such file or directory: path: "));
	}
	catch(const std::exception& e){
		FAIL() << "unknown exception thrown: load config file";
	}
}

TEST(hal_configFileTest, scaleOffsetAnalogOut0){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableOutput<double> &aOut = *hal.getRealOutput("aOut0");
		
		EXPECT_NEAR(0.000305180437934, aOut.getScale(), 0.000000000000001);
		EXPECT_NEAR(-10.0, aOut.getOffset(), 000000000000001);
		
		hal.releaseOutput("aOut0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, scaleOffsetAnalogOut1){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableOutput<double> &aOut = *hal.getRealOutput("aOut1");
		
		EXPECT_NEAR(0.000305180437934, aOut.getScale(), 0.000000000000001);
		EXPECT_NEAR(-10.0, aOut.getOffset(), 000000000000001);
		
		hal.releaseOutput("aOut1");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, scaleOffsetAnalogOut2){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableOutput<double> &aOut = *hal.getRealOutput("aOut2");
		
		EXPECT_NEAR(0.000610360875868, aOut.getScale(), 0.000000000000001);
		EXPECT_NEAR(-20.0, aOut.getOffset(), 000000000000001);
		
		hal.releaseOutput("aOut2");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, scaleOffsetAnalogOut3){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableOutput<double> &aOut = *hal.getRealOutput("aOut3");
		
		EXPECT_NEAR(0.000152590218967, aOut.getScale(), 0.000000000000001);
		EXPECT_NEAR(-4.0, aOut.getOffset(), 000000000000001);
		
		hal.releaseOutput("aOut3");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, rangeAnalogOut0){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableOutput<double> &aOut = *hal.getRealOutput("aOut0");
		
		EXPECT_NEAR(49151.25, aOut.getMaxOut(), 0.000000000000001);
		EXPECT_NEAR(16383.75, aOut.getMinOut(), 0.000000000000001);
		
		hal.releaseOutput("aOut0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, rangeAnalogOut1){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableOutput<double> &aOut = *hal.getRealOutput("aOut1");
		
		EXPECT_NEAR(65535, aOut.getMaxOut(), 0.000000000000001);
		EXPECT_NEAR(0, aOut.getMinOut(), 0.000000000000001);
		
		hal.releaseOutput("aOut1");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, rangeAnalogOut2){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableOutput<double> &aOut = *hal.getRealOutput("aOut2");
		
		EXPECT_NEAR(40959.375, aOut.getMaxOut(), 0.000000000000001);
		EXPECT_NEAR(24575.625, aOut.getMinOut(), 0.000000000000001);
		
		hal.releaseOutput("aOut2");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, rangeAnalogOut3){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableOutput<double> &aOut = *hal.getRealOutput("aOut3");
		
		EXPECT_NEAR(58981.5, aOut.getMaxOut(), 0.000000000000001);
		EXPECT_NEAR(0, aOut.getMinOut(), 0.000000000000001);
		
		hal.releaseOutput("aOut3");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, scaleAnalogIn0){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableInput<double> &aIn0 = *hal.getRealInput("aIn0");
		
		EXPECT_NEAR(3116.3265306122448979, aIn0.getScale(), 0.000000000000001);
		EXPECT_NEAR(32770, aIn0.getOffset(), 000000000000001);
		
		hal.releaseInput("aIn0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}

TEST(hal_configFileTest, rangeAnalogIn0){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::ScalableInput<double> &aIn0 = *hal.getRealInput("aIn0");
		
		EXPECT_NEAR(10.0, aIn0.getMaxIn(), 0.000000000000001);
		EXPECT_NEAR(-10.0, aIn0.getMinIn(), 0.000000000000001);
		
		hal.releaseInput("aIn0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(const std::exception& e){
		FAIL() << e.what();
	}
}
