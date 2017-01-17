#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::hal;

TEST(ConfigFileLoadTest, noFile){
	HAL& hal = HAL::instance();
	
// 	try{
		hal.readConfigFromFile("");
// 	}
// 	catch(eeros::EEROSException const & err){
// 	      EXPECT_EQ(err.what(), std::string("No such file"));
// 	}
// 	catch(...){
// 		FAIL() << "Expected no such file";
// 	}
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