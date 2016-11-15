#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <string>
#include <iostream>

using namespace eeros;
using namespace eeros::logger;

int main() {
	// Create and initialize logger
	StreamLogWriter w(std::cout);
//	SysLogWriter w("LoggerTest1");
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show(~0);
	
	log.info() << "Logger Test 1 started...";
	
	log.trace() << "This is a debug message...";
	log.info() << "This is a info message...";
	log.warn() << "This is a warning message...";
	log.error() << "This is a error message...";
	log.fatal() << "This is a fatal error message...";
	
	log.info() << "First line" << endl << "second line";
	
	int i = -4458;
	uint32_t u = 3423;
	char c = 'c';
	double d = 1.23456789;
	std::string s("Hello World");
	
	log.info() << "Integer: " << i;
	log.info() << "Unsigned integer: " << u;
	log.info() << "Double: " << d;
	log.info() << "String: " << s;
	
	log.info() << "Logger Test 1 finished...";
}
