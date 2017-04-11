#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <gtest/gtest.h>
#include <TestVariables.hpp>

using namespace eeros;
using namespace eeros::hal;

TEST(halHalManagerTest, availableDigInOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn");
		eeros::hal::Output<bool> &io1 = *hal.getLogicOutput("io1");
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut");
		eeros::hal::Output<bool> &ioOut4 = *hal.getLogicOutput("ioOut4");
		
		eeros::hal::Output<double> &dac1 = *hal.getScalableOutput("aOut0");
		
		eeros::hal::Input<double> &aIn0 = *hal.getScalableInput("aIn0");
		
		hal.releaseInput("ioIn");
		hal.releaseOutput("io1");
		hal.releaseOutput("ioOut");
		hal.releaseOutput("ioOut4");
		hal.releaseOutput("aOut0");
		hal.releaseInput("aIn0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, notAvailableDigInOut){
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

TEST(halHalManagerTest, emptySignalIdLogicIn){
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

TEST(halHalManagerTest, emptySignalIdLogicOut){
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

TEST(halHalManagerTest, emptySignalIdRealIn){
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

TEST(halHalManagerTest, emptySignalIdRealOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &en = *hal.getScalableOutput("");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongTypeRealIn){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &ioIn = *hal.getScalableInput("ioIn");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'ioIn' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongTypeRealOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getScalableOutput("ioOut");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output 'ioOut' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongTypeLogicIn){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &dac = *hal.getLogicInput("aIn0");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'aIn0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongTypeLogicOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("aOut0");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output 'aOut0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongDirectionLogicIn){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioOut");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'ioOut' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongDirectionLogicOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioIn");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output 'ioIn' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongDirectionRealIn){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn0 = *hal.getScalableInput("aOut0");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'aOut0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongDirectionRealOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getScalableOutput("aIn0");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output 'aIn0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn");
		eeros::hal::Input<bool> &ioIn2 = *hal.getLogicInput("ioIn");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'ioIn' is exclusive reserved!"));
		hal.releaseInput("ioIn");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn", false);
		eeros::hal::Input<bool> &ioIn2 = *hal.getLogicInput("ioIn", false);
		eeros::hal::Input<bool> &ioIn3 = *hal.getLogicInput("ioIn", false);
		
		hal.releaseInput("ioIn");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn", false);
		
		hal.releaseInput("ioIn");
		
		eeros::hal::Input<bool> &ioInEx = *hal.getLogicInput("ioIn");
		
		hal.releaseInput("ioIn");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveInputFail){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn", false);
		eeros::hal::Input<bool> &ioInEx = *hal.getLogicInput("ioIn");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'ioIn' is already claimed as non-exclusive input!"));
		hal.releaseInput("ioIn");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedNonExclusiveInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn");
		eeros::hal::Input<bool> &ioIn2 = *hal.getLogicInput("ioIn", false);
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'ioIn' is exclusive reserved!"));
		hal.releaseInput("ioIn");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut");
		eeros::hal::Output<bool> &ioOut2 = *hal.getLogicOutput("ioOut");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output 'ioOut' is exclusive reserved!"));
		hal.releaseOutput("ioOut");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut", false);
		eeros::hal::Output<bool> &ioOut2 = *hal.getLogicOutput("ioOut", false);
		eeros::hal::Output<bool> &ioOut3 = *hal.getLogicOutput("ioOut", false);
		
		hal.releaseOutput("ioOut");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut", false);
		
		hal.releaseOutput("ioOut");
		
		eeros::hal::Output<bool> &ioOut2 = *hal.getLogicOutput("ioOut");
		
		hal.releaseOutput("ioOut");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveOutputFail){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut", false);
		eeros::hal::Output<bool> &ioOut2 = *hal.getLogicOutput("ioOut");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output 'ioOut' is already claimed as non-exclusive output!"));
		hal.releaseOutput("ioOut");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedNonExclusiveOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut");
		eeros::hal::Output<bool> &ioOut2 = *hal.getLogicOutput("ioOut", false);
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output 'ioOut' is exclusive reserved!"));
		hal.releaseOutput("ioOut");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedRealInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn0 = *hal.getScalableInput("aIn0");
		eeros::hal::Input<double> &aIn2 = *hal.getScalableInput("aIn0");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'aIn0' is exclusive reserved!"));
		hal.releaseInput("aIn0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn0 = *hal.getScalableInput("aIn0", false);
		eeros::hal::Input<double> &aIn2 = *hal.getScalableInput("aIn0", false);
		eeros::hal::Input<double> &aIn3 = *hal.getScalableInput("aIn0", false);
		
		hal.releaseInput("aIn0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn0 = *hal.getScalableInput("aIn0", false);
		
		hal.releaseInput("aIn0");
		
		eeros::hal::Input<double> &aIn2 = *hal.getScalableInput("aIn0");
		
		hal.releaseInput("aIn0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveRealInputFail){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn0 = *hal.getScalableInput("aIn0", false);
		eeros::hal::Input<double> &aIn2 = *hal.getScalableInput("aIn0");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'aIn0' is already claimed as non-exclusive input!"));
		hal.releaseInput("aIn0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedNonExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn0 = *hal.getScalableInput("aIn0");
		eeros::hal::Input<double> &aIn2 = *hal.getScalableInput("aIn0", false);
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'aIn0' is exclusive reserved!"));
		hal.releaseInput("aIn0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getScalableOutput("aOut0");
		eeros::hal::Output<double> &aOut2 = *hal.getScalableOutput("aOut0");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output 'aOut0' is exclusive reserved!"));
		hal.releaseOutput("aOut0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getScalableOutput("aOut0", false);
		eeros::hal::Output<double> &aOut2 = *hal.getScalableOutput("aOut0", false);
		eeros::hal::Output<double> &aOut3 = *hal.getScalableOutput("aOut0", false);
		
		hal.releaseOutput("aOut0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getScalableOutput("aOut0", false);
		
		hal.releaseOutput("aOut0");
		
		eeros::hal::Output<double> &aOut2 = *hal.getScalableOutput("aOut0");
		
		hal.releaseOutput("aOut0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveRealOutputFail){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getScalableOutput("aOut0", false);
		eeros::hal::Output<double> &aOut2 = *hal.getScalableOutput("aOut0");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output 'aOut0' is already claimed as non-exclusive output!"));
		hal.releaseOutput("aOut0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedNonExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getScalableOutput("aOut0");
		eeros::hal::Output<double> &aOut2 = *hal.getScalableOutput("aOut0", false);
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output 'aOut0' is exclusive reserved!"));
		hal.releaseOutput("aOut0");
	}
	catch(...){
		FAIL();
	}
}
