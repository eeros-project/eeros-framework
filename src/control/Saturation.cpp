#include <eeros/control/Saturation.hpp>

using namespace eeros::control;

Saturation::Saturation(double lim, sigdim_t dim) : Block1i1o(dim), lowLimit(dim)u, pLimit(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->upLimit[i] = lim;
		this->lowLimit[i] = -lim;
	}
	this->enabled = true;
}

Saturation::Saturation(const double lim[], sigdim_t dim) : Block1i1o(dim), lowLimit(dim), upLimit(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->upLimit[i] = lim;
		this->lowLimit[i] = -lim;
	}
	this->enabled = true;
}

Saturation::Saturation(std::vector<double> lim, sigdim_t dim) : Block1i1o(dim), lowLimit(dim), upLimit(dim) {
	if(lim.size() == dim) {
		for(sigdim_t i = 0; i < dim; i++) {
			this->upLimit[i] = lim;
			this->lowLimit[i] = -lim;
		}
		this->enabled = true;
	}
}

Saturation::Saturation(double upLim, double lowLim, sigdim_t dim) : Block1i1o(dim), lowLimit(dim), upLimit(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->upLimit[i] = upLim;
		this->lowLimit[i] = lowLim;
	}
	this->enabled = true;
}

Saturation::Saturation(const double upLim[], const double lowLim[], sigdim_t dim) : Block1i1o(dim), lowLimit(dim), upLimit(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->upLimit[i] = upLim;
		this->lowLimit[i] = lowLim;
	}
	this->enabled = true;
}

Saturation::Saturation(std::vector<double> upLim, std::vector<double> lowLim, sigdim_t dim) : Block1i1o(dim), lowLimit(dim), upLimit(dim) {
	if(upLim.size() == lowLim.size() == dim) {
		for(sigdim_t i = 0; i < dim; i++) {
			this->upLimit[i] = upLim;
			this->lowLimit[i] = lowLim;
		}
		this->enabled = true;
	}
}

Saturation::~Saturation() {
	// nothing to do
}

void Saturation::run() {
	for(int i = 0; i < out.getDimension(); i++) {
		if(enabled){
			if(in.getValue(i)>upLimit(i))
				out.setValue(upLimit(i), i);
			else if (in.getValue(i)<lowLimit(i))
				out.setValue(lowLimit(i), i);
			else
				out.setValue(in.getValue(i), i);
		}
		else {
			out.setValue(in.getValue(i), i);	
		}
		out.setTimeStamp(in.getTimestamp());
	}
}

void Saturation::enable() {
	this->enabled = true;
}

void Saturation::disable() {
	this->enabled = false;
}

void Saturation::setSaturationLimit(double lim) {
	for(sigdim_t i = 0; i < lowLim.size(); i++) {
		this->upLimit[i] = lim;
		this->lowLimit[i] = -lim;
	}
}

void Saturation::setSaturationLimit(double lowLim, double upLim) {
	for(sigdim_t i = 0; i < lowLim.size(); i++) {
		this->upLimit[i] = upLim;
		this->lowLimit[i] = lowLim;
	}
}

void Saturation::setSaturationLimit(sigindex_t index, double lim) {
	if(index < this->lim.size()) {
		upLimit[index] = lim;
		lowLimit[index] = -lim;
	}
}

void Saturation::setSaturationLimit(sigindex_t index, double lowLim, double upLim) {
	if(index < this->upLim.size()) {
		upLimit[index] = upLim;
		lowLimit[index] = lowLim;
	}
}


