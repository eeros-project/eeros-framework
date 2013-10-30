#include <eeros/control/Block1o.hpp>

using namespace eeros::control;

Block1o::Block1o(sigdim_t dim) : out(dim) { }

RealSignalOutput& Block1o::getOut() {
	return out;
}