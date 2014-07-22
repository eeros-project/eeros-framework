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
	
	int i = -1;
	unsigned int u = 5;
	float f = 1.23456789;
	double d = 2 * f;
	char z = 'z';
	std::string s("Hello World");
	Matrix<4, 1, int> v; v << 1, 2, 3, 4;
	Matrix<3, 3, int> m; m << 1, 4, 7,
	                          2, 5, 8,
	                          3, 6, 9;
	CoordinateSystem a("a");
	CoordinateSystem b("b");
	CoordinateSystem c("c");
	Frame fab(a, b);
	Frame fba(b, a);
	Frame fac(a, c);
	Frame fbc(b, c);
	
	std::cout << v << std::endl;
	std::cout << m << std::endl;
	
	log.info() << "Integer:           " << i;
	log.info() << "Unsigned Integer:  " << u;
	log.info() << "Float:             " << f;
	log.info() << "Double:            " << d;
	log.info() << "Char:              " << z;
	log.info() << "String:            " << s;
// 	log.info() << "Vector:            " << v; // not yet implemented
// 	log.info() << "Matrix 4x4:        " << m;
	
	if(Frame::getFrame(a, b) == nullptr) log.error() << "Error: Frame(a, b) not found!";
	
	
	
	log.info() << "Martin Test 3 finished...";
}
