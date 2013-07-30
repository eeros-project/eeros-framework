#include <eeros/control/Block1i3o.hpp>
#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/RealSignalInput.hpp>

Block1i3o::Block1i3o(sigdim_t dim) : out1(dim),out2(dim),out3(dim) { }

RealSignalInput& Block1i3o::getIn() {
	return in;
}

RealSignalOutput& Block1i3o::getOut1() {
	return out1;
}

RealSignalOutput& Block1i3o::getOut2() {
	return out2;
}

RealSignalOutput& Block1i3o::getOut3() {
	return out3;
}
