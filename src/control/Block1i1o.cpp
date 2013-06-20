#include <eeros/control/Block1i1o.hpp>
#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/RealSignalInput.hpp>

Block1i1o::Block1i1o(sigdim_t dim) : out(dim) { }

RealSignalInput& Block1i1o::getIn() {
	return in;
}

RealSignalOutput& Block1i1o::getOut() {
	return out;
}