#include <eeros/control/Block1i.hpp>

Block1i::Block1i() : in() { }

RealSignalInput& Block1i::getIn() {
	return in;
}