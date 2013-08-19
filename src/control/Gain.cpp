#include <eeros/control/Gain.hpp>

Gain::Gain(sigdim_t dim) : Block1i1o(dim) {
	this->gain = new double[dim];
	for(int i = 0; i < dim; i++) {
		this->gain[i] = 1.0;
	}
	this->enabled = true;
}

Gain::Gain(sigdim_t dim, double gain[]) : Block1i1o(dim) {
	if(sizeof(gain) / sizeof(gain[0]) == dim) { // TODO check what happens if lenght of array is 0!
		this->gain = new double[dim];
		for(int i = 0; i < dim; i++)
		{
			this->gain[i] = gain[i];
		}
		this->enabled = true;
	}
}



Gain::~Gain() {
// 	delete this->out;
// 	delete this->gain;
}

void Gain::run() {
	for(int i = 0; i < out.getDimension(); i++)
	{
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

void Gain::setGain(double gain, sigindex_t index) {
	this->gain[index] = gain;
}
