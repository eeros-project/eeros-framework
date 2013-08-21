#include <eeros/control/Gain.hpp>

Gain::Gain(double c, sigdim_t dim) : Block1i1o(dim), gain(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->gain[i] = c;
	}
	this->enabled = true;
}

Gain::Gain(const double gain[], sigdim_t dim) : Block1i1o(dim), gain(dim) {
	if(sizeof(gain) / sizeof(gain[0]) == dim) { // TODO check what happens if lenght of array is 0!
		for(sigdim_t i = 0; i < dim; i++) {
			this->gain[i] = gain[i];
		}
		this->enabled = true;
	}
}

Gain::Gain(std::vector<double> gain, sigdim_t dim) : Block1i1o(dim), gain(dim) {
	if(gain.size() == dim) {
		for(sigdim_t i = 0; i < dim; i++) {
			this->gain[i] = gain[i];
		}
		this->enabled = true;
	}
}

Gain::~Gain() {
	// nothing to do
}

void Gain::run() {
	for(int i = 0; i < out.getDimension(); i++) {
		if(enabled) out.setValue(in.getValue(i) * gain[i], i);
		else out.setValue(in.getValue(i), i);
		out.setTimeStamp(in.getTimestamp());
	}
}

void Gain::enable() {
	this->enabled = true;
}

void Gain::disable() {
	this->enabled = true;
}

void Gain::setGain(sigindex_t index, double c) {
	if(index < this->gain.size()) {
		gain[index] = c;
	}
}

void Gain::setGain(double c) {
	for(sigdim_t i = 0; i < gain.size(); i++) {
		this->gain[i] = c;
	}
}
