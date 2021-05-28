#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>
#include <TestVariables.hpp>
#include <eeros/hal/ScalableOutput.hpp>

using namespace eeros;
using namespace eeros::hal;

TEST(halLoadConfigFileTest, noFile){
  HAL& hal = HAL::instance();
  
  try{
    hal.readConfigFromFile("");
  }
  catch(eeros::Fault const & err){
        EXPECT_EQ(err.what(), std::string("no configuration file given"));
  }
  catch(const std::exception& e){
    FAIL() << "unknown exception thrown: load config file";
  }
}

TEST(halConfigFileTest, scaleOffsetAnalogOut0){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut0");
    
    EXPECT_NEAR(0.000305180437934, aOut.getScale(), 0.000000000000001);
    EXPECT_NEAR(-10.0, aOut.getOffset(), 000000000000001);
    
    hal.releaseOutput("aOut0");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, scaleOffsetAnalogOut1){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut1");
    
    EXPECT_NEAR(0.000305180437934, aOut.getScale(), 0.000000000000001);
    EXPECT_NEAR(-10.0, aOut.getOffset(), 000000000000001);
    
    hal.releaseOutput("aOut1");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, scaleOffsetAnalogOut2){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut2");
    
    EXPECT_NEAR(0.000610360875868, aOut.getScale(), 0.000000000000001);
    EXPECT_NEAR(-20.0, aOut.getOffset(), 000000000000001);
    
    hal.releaseOutput("aOut2");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, scaleOffsetAnalogOut3){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut3");
    
    EXPECT_NEAR(0.000152590218967, aOut.getScale(), 0.000000000000001);
    EXPECT_NEAR(-4.0, aOut.getOffset(), 000000000000001);
    
    hal.releaseOutput("aOut3");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, rangeAnalogOut0){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut0");
    
    EXPECT_NEAR(49151.25, aOut.getMaxOut(), 0.000000000000001);
    EXPECT_NEAR(16383.75, aOut.getMinOut(), 0.000000000000001);
    
    hal.releaseOutput("aOut0");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, rangeAnalogOut1){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut1");
    
    EXPECT_NEAR(65535, aOut.getMaxOut(), 0.000000000000001);
    EXPECT_NEAR(0, aOut.getMinOut(), 0.000000000000001);
    
    hal.releaseOutput("aOut1");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, rangeAnalogOut2){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut2");
    
    EXPECT_NEAR(40959.375, aOut.getMaxOut(), 0.000000000000001);
    EXPECT_NEAR(24575.625, aOut.getMinOut(), 0.000000000000001);
    
    hal.releaseOutput("aOut2");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, rangeAnalogOut3){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut3");
    
    EXPECT_NEAR(58981.5, aOut.getMaxOut(), 0.000000000000001);
    EXPECT_NEAR(0, aOut.getMinOut(), 0.000000000000001);
    
    hal.releaseOutput("aOut3");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, scaleAnalogIn0){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableInput<double> &aIn0 = *hal.getScalableInput("aIn0");
    
    EXPECT_NEAR(3116.3265306122448979, aIn0.getScale(), 0.000000000000001);
    EXPECT_NEAR(32770, aIn0.getOffset(), 000000000000001);
    
    hal.releaseInput("aIn0");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, rangeAnalogIn0){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableInput<double> &aIn0 = *hal.getScalableInput("aIn0");
    
    EXPECT_NEAR(10.0, aIn0.getMaxIn(), 0.000000000000001);
    EXPECT_NEAR(-10.0, aIn0.getMinIn(), 0.000000000000001);
    
    hal.releaseInput("aIn0");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, scaleOffsetAnalogOut4){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut4");
    
    EXPECT_NEAR(20.0/1000, aOut.getScale(), 0.000000000000001);
    EXPECT_NEAR(-10, aOut.getOffset(), 000000000000001);
    EXPECT_NEAR(0, aOut.getMinOut(), 000000000000001);
    EXPECT_NEAR(1000, aOut.getMaxOut(), 000000000000001);
    
    hal.releaseOutput("aOut4");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, scaleOffsetAnalogOut5){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut5");
    
    EXPECT_NEAR(20.0/800, aOut.getScale(), 0.000000000000001);
    EXPECT_NEAR(-10, aOut.getOffset(), 000000000000001);
    EXPECT_NEAR(0, aOut.getMinOut(), 000000000000001);
    EXPECT_NEAR(800, aOut.getMaxOut(), 000000000000001);
    
    hal.releaseOutput("aOut5");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, scaleOffsetAnalogOut6){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut6");
    
    EXPECT_NEAR(20.0/800, aOut.getScale(), 0.000000000000001);
    EXPECT_NEAR(-15, aOut.getOffset(), 000000000000001);
    EXPECT_NEAR(200, aOut.getMinOut(), 000000000000001);
    EXPECT_NEAR(1000, aOut.getMaxOut(), 000000000000001);
    
    hal.releaseOutput("aOut6");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}

TEST(halConfigFileTest, scaleOffsetAnalogOut7){
  HAL& hal = HAL::instance();
  
  try{
    eeros::hal::ScalableOutput<double> &aOut = *hal.getScalableOutput("aOut7");
    
    EXPECT_NEAR(20.0/600, aOut.getScale(), 0.000000000000001);
    EXPECT_NEAR(-16.667, aOut.getOffset(), 000000000000001);
    EXPECT_NEAR(200, aOut.getMinOut(), 000000000000001);
    EXPECT_NEAR(800, aOut.getMaxOut(), 000000000000001);
    
    hal.releaseOutput("aOut7");
  }
  catch(eeros::Fault const & err){
    FAIL() << err.what();
  }
  catch(const std::exception& e){
    FAIL() << e.what();
  }
}


