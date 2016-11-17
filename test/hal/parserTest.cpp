#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/JsonParser.hpp>
#include <eeros/hal/HAL.hpp>

using namespace eeros::logger;
using namespace eeros::hal;

int main(){
	// Create and initialize logger
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "ParserTest started...";
  
	HAL& hal = HAL::instance();
	hal.readConfigFromFile("../../../../eeros-framework/test/hal/HALConfigExample.json");
	
}