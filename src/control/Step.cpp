#include <eeros/control/Step.hpp>

Step::Step(double initValue, double stepHeight, double delayTime, sigdim_t dim) : Block1o(dim), initValue(dim), stepHeight(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->initValue[i] = initValue;
		this->stepHeight[i] = stepHeight;
	}
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::Step(const double initValue[],const  double stepHeight[], double delayTime, sigdim_t dim) : Block1o(dim), initValue(dim), stepHeight(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->initValue[i] = initValue[i];
		this->stepHeight[i] = stepHeight[i];
	}
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::Step(const std::vector<double> initValues, const std::vector<double> stepHeight, double delayTime, sigdim_t dim) : Block1o(dim), initValue(dim), stepHeight(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->initValue[i] = initValue[i];
		this->stepHeight[i] = stepHeight[i];
	}
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::~Step() {
	// nothing to do
}

void Step::run() {
	if (first) {
		stepTime += System::getTime();
		first = false;
		for(int i = 0; i < out.getDimension(); i++) {
			out.setValue(initValue[i], i);
		}
		
	}
	if(!stepDone && System::getTime() >= stepTime) {
		for(int i = 0; i < out.getDimension(); i++) {
			out.setTimeStamp(System::getTimeNs());
			out.setValue(initValue[i] + stepHeight[i], i);
		}
		stepDone = true;
	}
	else {
		for(int i = 0; i < out.getDimension(); i++) {
			out.setTimeStamp(System::getTimeNs());
		}
	}
}

void Step::reset() {
	stepDone = false;
	first = true;
}

void Step::setinitValue(double initValue) {
	for(sigindex_t i = 0; i < this->initValue.size(); i++) {
		this->initValue[i] = initValue;
	}
}

void Step::setinitValue(sigindex_t index, double initValue) {
	this->initValue[index] = initValue;
}

void Step::setStepHeight(double stepHeight) {
	for(sigindex_t i = 0; i < this->stepHeight.size(); i++) {
		this->stepHeight[i] = stepHeight;
	}
}

void Step::setStepHeight(sigindex_t index, double stepHeight) {
	this->stepHeight[index] = stepHeight;
}

void Step::setDelayTime(double delayTime) {
	this->stepTime = delayTime;
}
