#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/JsonParser.hpp>

using namespace eeros::logger;
using namespace eeros::hal;

int main(){
	// Create and initialize logger
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "ParserTest started...";
  
	JsonParser parser = JsonParser("../../../../eeros-framework/test/hal/HALConfigExample.json");
	parser.createHalObjects();
}