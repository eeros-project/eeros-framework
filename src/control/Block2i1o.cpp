#include <eeros/control/Block2i1o.hpp>
#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/RealSignalInput.hpp>

Block2i1o::Block2i1o(sigdim_t dim) : out(dim) { }

RealSignalInput& Block2i1o::getIn1() {
	return in1;
}

RealSignalInput& Block2i1o::getIn2() {
	return in2;
}

RealSignalOutput& Block2i1o::getOut() {
	return out;
}
