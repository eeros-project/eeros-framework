#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>
#include <TestVariables.hpp>

using namespace eeros;
using namespace eeros::hal;

TEST(halHalManagerTest, availableDigInOut){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("ioIn");
		hal.getLogicOutput("io1");
		hal.getLogicOutput("ioOut");
		hal.getLogicOutput("ioOut4");
		
		hal.getScalableOutput("aOut0");
		
		hal.getScalableInput("aIn0");
		
		hal.releaseInput("ioIn");
		hal.releaseOutput("io1");
		hal.releaseOutput("ioOut");
		hal.releaseOutput("ioOut4");
		hal.releaseOutput("aOut0");
		hal.releaseInput("aIn0");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, notAvailableDigInOut){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("enable56");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'enable56' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, emptySignalIdLogicIn){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, emptySignalIdLogicOut){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicOutput("");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, emptySignalIdRealIn){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, emptySignalIdRealOut){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableOutput("");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system output '' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongTypeRealIn){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableInput("ioIn");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system input 'ioIn' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongTypeRealOut){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableOutput("ioOut");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system output 'ioOut' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongTypeLogicIn){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("aIn0");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'aIn0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongTypeLogicOut){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicOutput("aOut0");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output 'aOut0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongDirectionLogicIn){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("ioOut");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Logic system input 'ioOut' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongDirectionLogicOut){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicOutput("ioIn");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Logic system output 'ioIn' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongDirectionRealIn){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableInput("aOut0");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system input 'aOut0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, wrongDirectionRealOut){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableOutput("aIn0");
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system output 'aIn0' not found!"));
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedInput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("ioIn");
		hal.getLogicInput("ioIn");
		
		FAIL();
	}
	catch(eeros::Fault const & err){
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
		hal.getLogicInput("ioIn", false);
		hal.getLogicInput("ioIn", false);
		hal.getLogicInput("ioIn", false);
		
		hal.releaseInput("ioIn");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveInput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("ioIn", false);
		
		hal.releaseInput("ioIn");
		
		hal.getLogicInput("ioIn");
		
		hal.releaseInput("ioIn");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveInputFail){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicInput("ioIn", false);
		hal.getLogicInput("ioIn");
		
		FAIL();
	}
	catch(eeros::Fault const & err){
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
		hal.getLogicInput("ioIn");
		hal.getLogicInput("ioIn", false);
		
		FAIL();
	}
	catch(eeros::Fault const & err){
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
		hal.getLogicOutput("ioOut");
		hal.getLogicOutput("ioOut");
		
		FAIL();
	}
	catch(eeros::Fault const & err){
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
		hal.getLogicOutput("ioOut", false);
		hal.getLogicOutput("ioOut", false);
		hal.getLogicOutput("ioOut", false);
		
		hal.releaseOutput("ioOut");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveOutput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicOutput("ioOut", false);
		
		hal.releaseOutput("ioOut");
		
		hal.getLogicOutput("ioOut");
		
		hal.releaseOutput("ioOut");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveOutputFail){
	HAL& hal = HAL::instance();
	
	try{
		hal.getLogicOutput("ioOut", false);
		hal.getLogicOutput("ioOut");
		
		FAIL();
	}
	catch(eeros::Fault const & err){
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
		hal.getLogicOutput("ioOut");
		hal.getLogicOutput("ioOut", false);
		
		FAIL();
	}
	catch(eeros::Fault const & err){
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
		hal.getScalableInput("aIn0");
		hal.getScalableInput("aIn0");
		
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system input 'aIn0' is exclusive reserved!"));
		hal.releaseInput("aIn0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableInput("aIn0", false);
		hal.getScalableInput("aIn0", false);
		hal.getScalableInput("aIn0", false);
		
		hal.releaseInput("aIn0");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableInput("aIn0", false);
		
		hal.releaseInput("aIn0");
		
		hal.getScalableInput("aIn0");
		
		hal.releaseInput("aIn0");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveRealInputFail){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableInput("aIn0", false);
		hal.getScalableInput("aIn0");
		
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system input 'aIn0' is already claimed as non-exclusive input!"));
		hal.releaseInput("aIn0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedNonExclusiveRealInput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableInput("aIn0");
		hal.getScalableInput("aIn0", false);
		
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system input 'aIn0' is exclusive reserved!"));
		hal.releaseInput("aIn0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableOutput("aOut0");
		hal.getScalableOutput("aOut0");
		
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system output 'aOut0' is exclusive reserved!"));
		hal.releaseOutput("aOut0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableOutput("aOut0", false);
		hal.getScalableOutput("aOut0", false);
		hal.getScalableOutput("aOut0", false);
		
		hal.releaseOutput("aOut0");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableOutput("aOut0", false);
		
		hal.releaseOutput("aOut0");
		
		hal.getScalableOutput("aOut0");
		
		hal.releaseOutput("aOut0");
	}
	catch(eeros::Fault const & err){
		FAIL() << err.what();
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimNonExclusiveThenExclusiveRealOutputFail){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableOutput("aOut0", false);
		hal.getScalableOutput("aOut0");
		
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system output 'aOut0' is already claimed as non-exclusive output!"));
		hal.releaseOutput("aOut0");
	}
	catch(...){
		FAIL();
	}
}

TEST(halHalManagerTest, claimReservedNonExclusiveRealOutput){
	HAL& hal = HAL::instance();
	
	try{
		hal.getScalableOutput("aOut0");
		hal.getScalableOutput("aOut0", false);
		
		FAIL();
	}
	catch(eeros::Fault const & err){
		EXPECT_EQ(err.what(), std::string("Scalable system output 'aOut0' is exclusive reserved!"));
		hal.releaseOutput("aOut0");
	}
	catch(...){
		FAIL();
	}
}
