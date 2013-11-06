#include <eeros/core/EEROSException.hpp>

using namespace eeros;

EEROSException::EEROSException(std::string message) : message(message) { }

EEROSException::~EEROSException() throw() { }

const char* EEROSException::what() const throw() {
	return message.c_str();
}
