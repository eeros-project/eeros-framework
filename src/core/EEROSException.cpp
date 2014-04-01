#include <eeros/core/EEROSException.hpp>

using namespace eeros;

EEROSException::EEROSException() { }

EEROSException::EEROSException(std::string m) : message(m) {
	
}

EEROSException::~EEROSException() throw() { }

const char* EEROSException::what() const throw() {
	return message.c_str();
}
