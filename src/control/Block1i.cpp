#include <eeros/control/Block1i.hpp>

using namespace eeros::control;

Block1i::Block1i() : in() { }

RealSignalInput& Block1i::getIn() {
	return in;
}