#include <eeros/control/Sum.hpp>

Sum::Sum(uint8_t nofInputs, sigdim_t dim) : out(dim), in(nofInputs), negated(nofInputs) {
	
}

void Sum::run() {
	double sum;
	for(sigdim_t i = 0; i < out.getDimension(); i++) {
		sum = 0;
		for(uint8_t j = 0; j < in.size(); j++) {
			if(negated[j]) sum -= in[j].getValue(i);
			else sum += in[j].getValue(i);
		}
		
		out.setValue(sum, i);
		out.setTimeStamp(in[0].getTimestamp(i));
	}
}
RealSignalInput& Sum::getIn(uint8_t input) {
	return in[input];
}

RealSignalOutput& Sum::getOut() {
	return out;
}

void Sum::negateInput(uint8_t input) {
	negated[input] = true;
}
