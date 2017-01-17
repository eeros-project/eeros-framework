#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::hal;

TEST(ConfigFileLoadTest, noFile){
	HAL& hal = HAL::instance();
	
	try{
		hal.readConfigFromFile("");
	}
	catch(eeros::EEROSException const & err){
	      EXPECT_EQ(err.what(), std::string("cannot open file : No such file or directory"));
	}
	catch(const std::exception& e){
		FAIL() << "unknown exception thrown: load config file";
	}
}

TEST(ConfigFileLoadTest, validFile){
	HAL& hal = HAL::instance();
	try{
		hal.readConfigFromFile("loadConfig.json");
	}
	catch(eeros::EEROSException const & err){
		FAIL();
	}
	catch(...){
		FAIL();
	}
	SUCCEED();
}


int main(int argc, char **argv){

	::testing::InitGoogleTest(&argc, argv);
	
	return RUN_ALL_TESTS();
  
	
}