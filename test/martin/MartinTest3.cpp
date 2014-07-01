#include <iostream>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/math/CoordinateSystem.hpp>
#include <eeros/math/Frame.hpp>

using namespace eeros::logger;
using namespace eeros::math;

int main() {
	// Create and initialize logger
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "Martin Test 3 started...";
	
	CoordinateSystem a("a");
	CoordinateSystem b("b");
	CoordinateSystem c("c");
	Frame fab(a, b);
	Frame fba(b, a);
	Frame fac(a, c);
	Frame fbc(b, c);
	
	if(Frame::getFrame(a, b) == nullptr) log.error() << "Error: Frame(a, b) not found!";
	
	log.info() << "Martin Test 3 finished...";
}
