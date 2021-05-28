#include <EerosEnvironmentInvalidConfig.hpp>
#include <eeros/hal/HAL.hpp>
#include <TestVariables.hpp>

using namespace eeros::test;
using namespace eeros::hal;

void EerosEnvironmentInvalidConfig::SetUp() {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  std::cout << "[----------] set-up invalid config EEROS test environment" << std::endl;
  HAL& hal = HAL::instance();
  try {
    hal.readConfigFromFile("invalidLib.json");
  }
  catch(eeros::Fault const & err){
    ASSERT_EQ(err.what(), std::string("libinvalideeros.so: cannot open shared object file: No such file or directory"));
    std::cout << "[----------] invalid library in config detected, test successful!" << std::endl;
  }
    
  if(libcomedi) {
    try {
      hal.readConfigFromFile("invalidScaleComedi.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("config of scale is invalid, id: 'dac'"));
      std::cout << "[----------] invalid scale in config detected, test successful!" << std::endl;
    }
    try {
      hal.readConfigFromFile("invalidChannelComedi.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("no valid key found: channela0"));
      std::cout << "[----------] invalid key in config detected, test successful!" << std::endl;
    }
    try {
      hal.readConfigFromFile("invalidTypeComedi.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("undefined type: DInvalidOut for digOutDummy"));
      std::cout << "[----------] invalid type in config detected, test successful!" << std::endl;
    }
  }
  else if(libflink) {
    try {
      hal.readConfigFromFile("invalidScaleFlink.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("config of scale is invalid, id: 'dac'"));
      std::cout << "[----------] invalid config detected, test successful!" << std::endl;
    }
    try {
      hal.readConfigFromFile("invalidChannelFlink.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("no valid key found: channela0"));
      std::cout << "[----------] invalid key in config detected, test successful!" << std::endl;
    }
    try {
      hal.readConfigFromFile("invalidTypeFlink.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("undefined type: DInvalidOut for digOutDummy"));
      std::cout << "[----------] invalid type in config detected, test successful!" << std::endl;
    }
  }
  else if(libsim) {
    try {
      hal.readConfigFromFile("invalidScaleSim.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("config of scale is invalid, id: 'dac'"));
      std::cout << "[----------] invalid scale in config detected, test successful!" << std::endl;
    }
    try {
      hal.readConfigFromFile("invalidChannelSim.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("no valid key found: channela0"));
      std::cout << "[----------] invalid key in config detected, test successful!" << std::endl;
    }
    try {
      hal.readConfigFromFile("invalidTypeSim.json");
    }
    catch(eeros::Fault const & err){
      ASSERT_EQ(err.what(), std::string("undefined type: DInvalidOut for digOutDummy"));
      std::cout << "[----------] invalid type in config detected, test successful!" << std::endl;
    }
  }
  else {
    throw eeros::Fault("no library selected with command line arguments (--library foo)");
  }
}

void EerosEnvironmentInvalidConfig::TearDown() {
  std::cout << "[----------] tear-down invalid config EEROS test environment" << std::endl;
}
