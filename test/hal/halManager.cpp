#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <gtest/gtest.h>
#include <TestVariables.hpp>

using namespace eeros;
using namespace eeros::hal;

TEST(hal_HalManagerTest, availableDigInOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &ioIn = *hal.getLogicInput("ioIn");
		eeros::hal::Output<bool> &io1 = *hal.getLogicOutput("io1");
		eeros::hal::Output<bool> &ioOut = *hal.getLogicOutput("ioOut");
		eeros::hal::Output<bool> &ioOut4 = *hal.getLogicOutput("ioOut4");
		
		eeros::hal::Output<double> &dac1 = *hal.getRealOutput("aOut0");
		
		eeros::hal::Input<double> &aIn = *hal.getRealInput("aIn");
		
		hal.releaseInput("ioIn");
		hal.releaseOutput("io1");
		hal.releaseOutput("ioOut");
		hal.releaseOutput("ioOut4");
		hal.releaseOutput("aOut0");
		hal.releaseInput("aIn");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, notAvailableDigInOut){
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

TEST(hal_HalManagerTest, emptySignalIdLogicIn){
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

TEST(hal_HalManagerTest, emptySignalIdLogicOut){
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

TEST(hal_HalManagerTest, emptySignalIdRealIn){
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

TEST(hal_HalManagerTest, emptySignalIdRealOut){
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

TEST(hal_HalManagerTest, wrongTypeRealIn){
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

TEST(hal_HalManagerTest, wrongTypeRealOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getRealOutput("ioOut");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output 'ioOut' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, wrongTypeLogicIn){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<bool> &dac = *hal.getLogicInput("aIn");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'aIn' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, wrongTypeLogicOut){
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

TEST(hal_HalManagerTest, wrongDirectionLogicIn){
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

TEST(hal_HalManagerTest, wrongDirectionLogicOut){
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

TEST(hal_HalManagerTest, wrongDirectionRealIn){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn = *hal.getRealInput("aOut0");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'aOut0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, wrongDirectionRealOut){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getRealOutput("aIn");
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system output 'aIn' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, claimReservedInput){
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

TEST(hal_HalManagerTest, claimNonExclusiveInput){
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

TEST(hal_HalManagerTest, claimNonExclusiveThenExclusiveInput){
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

TEST(hal_HalManagerTest, claimNonExclusiveThenExclusiveInputFail){
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

TEST(hal_HalManagerTest, claimReservedNonExclusiveInput){
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

TEST(hal_HalManagerTest, claimReservedOutput){
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

TEST(hal_HalManagerTest, claimNonExclusiveOutput){
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

TEST(hal_HalManagerTest, claimNonExclusiveThenExclusiveOutput){
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

TEST(hal_HalManagerTest, claimNonExclusiveThenExclusiveOutputFail){
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

TEST(hal_HalManagerTest, claimReservedNonExclusiveOutput){
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

TEST(hal_HalManagerTest, claimReservedRealInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn = *hal.getRealInput("aIn");
		eeros::hal::Input<double> &aIn2 = *hal.getRealInput("aIn");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'aIn' is exclusive reserved!"));
		hal.releaseInput("aIn");
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, claimNonExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn = *hal.getRealInput("aIn", false);
		eeros::hal::Input<double> &aIn2 = *hal.getRealInput("aIn", false);
		eeros::hal::Input<double> &aIn3 = *hal.getRealInput("aIn", false);
		
		hal.releaseInput("aIn");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, claimNonExclusiveThenExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn = *hal.getRealInput("aIn", false);
		
		hal.releaseInput("aIn");
		
		eeros::hal::Input<double> &aIn2 = *hal.getRealInput("aIn");
		
		hal.releaseInput("aIn");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, claimNonExclusiveThenExclusiveRealInputFail){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn = *hal.getRealInput("aIn", false);
		eeros::hal::Input<double> &aIn2 = *hal.getRealInput("aIn");
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'aIn' is already claimed as non-exclusive input!"));
		hal.releaseInput("aIn");
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, claimReservedNonExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Input<double> &aIn = *hal.getRealInput("aIn");
		eeros::hal::Input<double> &aIn2 = *hal.getRealInput("aIn", false);
		
		FAIL();
	}
	catch(eeros::EEROSException const & err){
		EXPECT_EQ(err.what(), std::string("Real system input 'aIn' is exclusive reserved!"));
		hal.releaseInput("aIn");
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, claimReservedRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getRealOutput("aOut0");
		eeros::hal::Output<double> &aOut2 = *hal.getRealOutput("aOut0");
		
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

TEST(hal_HalManagerTest, claimNonExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getRealOutput("aOut0", false);
		eeros::hal::Output<double> &aOut2 = *hal.getRealOutput("aOut0", false);
		eeros::hal::Output<double> &aOut3 = *hal.getRealOutput("aOut0", false);
		
		hal.releaseOutput("aOut0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, claimNonExclusiveThenExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getRealOutput("aOut0", false);
		
		hal.releaseOutput("aOut0");
		
		eeros::hal::Output<double> &aOut2 = *hal.getRealOutput("aOut0");
		
		hal.releaseOutput("aOut0");
	}
	catch(eeros::EEROSException const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(hal_HalManagerTest, claimNonExclusiveThenExclusiveRealOutputFail){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getRealOutput("aOut0", false);
		eeros::hal::Output<double> &aOut2 = *hal.getRealOutput("aOut0");
		
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

TEST(hal_HalManagerTest, claimReservedNonExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		eeros::hal::Output<double> &aOut = *hal.getRealOutput("aOut0");
		eeros::hal::Output<double> &aOut2 = *hal.getRealOutput("aOut0", false);
		
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
