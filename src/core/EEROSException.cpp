#include <eeros/core/EEROSException.hpp>

EEROSException::EEROSException(std::string message) : message(message) { }

EEROSException::~EEROSException() throw() { }

const char* EEROSException::what() {
	return message.c_str();
}
