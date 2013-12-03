#include <eeros/control/Switch.hpp>

using namespace eeros::control;

Switch::Switch(uint8_t nofInputs, sigdim_t dim) : out(dim), in(nofInputs), currentInput(0), maxDifference(dim) {
	
}

void Switch::run() {
	for(uint8_t i = 0; i < in.size(); i++) {
		out.setValue(in[currentInput].getValue(i), i);
		out.setTimeStamp(in[currentInput].getTimestamp(i), i);
	}
}
RealSignalInput& Switch::getIn(uint8_t input) {
	return in[input];
}

RealSignalOutput& Switch::getOut() {
	return out;
}

void Switch::setSwitchTolerance(double maxDifference) {
	for(sigdim_t i = 0; i < out.getDimension(); i++) {
		this->maxDifference[i] = maxDifference;
	}
}

void Switch::setSwitchTolerance(std::vector<double> maxDifference) {
	if(maxDifference.size() == out.getDimension()) {
		this->maxDifference = maxDifference;
	}
}

bool Switch::switchToInput(uint8_t newInput) {
	bool checkOk = true;
	double valA, valB;
	sigdim_t i = 0;
	while(checkOk && i < out.getDimension()) {
		valA = in[currentInput].getValue(i);
		valB = in[newInput].getValue(i);
		checkOk = (valB > valA - maxDifference[i] && valB < valA + maxDifference[i]);
		i++;
	}
	if(checkOk) {
		currentInput = newInput;
		return true;
	}
	return false;
}