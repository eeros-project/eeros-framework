#include <eeros/core/Fault.hpp>

using namespace eeros;

Fault::Fault() { }

Fault::Fault(std::string m) : message(m) { }

Fault::~Fault() throw() { }

const char* Fault::what() const throw() {
	return message.c_str();
}
