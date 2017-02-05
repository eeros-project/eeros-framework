#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <string>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::math;
using namespace eeros::safety;

int main() {
	// Create and initialize logger
	StreamLogWriter w(std::cout);
// 	SysLogWriter w("LoggerTest1");
	Logger::setDefaultWriter(&w);
	Logger log;
	w.show(LogLevel::TRACE);
// 	
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
	log.info() << "Char: " << c;
	log.info() << "Double: " << d;
	log.info() << "String: " << s;
	
	Matrix<4, 1, int> v; v << 1, 2, 3, 4;
	Matrix<3, 3, int> m; m << 1, 4, 7,
	                          2, 5, 8,
	                          3, 6, 9;
	log.info() << "Vector: " << v;
	log.info() << "Matrix: " << m;
	
	SafetyLevel level("test level");
	log.info() << "Safety level: " << level;
	SafetyEvent event("test event");
	log.info() << "Safety event: " << event;
	
	log.info() << "Logger Test 1 finished...";
}
