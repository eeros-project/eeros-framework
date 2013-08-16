#include <eeros/control/Step.hpp>

Step::Step(double initValue, double stepHeight, double delayTime) : Block1o(1) {
	this->initValue = new double[1];
	this->stepHeight = new double[1];
	this->initValue[0] = initValue;
	this->stepHeight[0] = stepHeight;
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::Step(sigdim_t dim, double initValue, double stepHeight, double delayTime) : Block1o(dim) {
	this->initValue = new double[dim];
	this->stepHeight = new double[dim];
	for(sigdim_t i = 0; i < dim; i++) {
		this->initValue[i] = initValue;
		this->stepHeight[i] = stepHeight;
	}
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::Step(sigdim_t dim, double initValue[], double stepHeight[], double delayTime) : Block1o(dim) {
	this->initValue = new double[dim];
	this->stepHeight = new double[dim];
	for(sigdim_t i = 0; i < dim; i++) {
		this->initValue[i] = initValue[i];
		this->stepHeight[i] = stepHeight[i];
	}
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::~Step() {
	delete this->initValue;
	delete this->stepHeight;
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
	this->setinitValue(initValue, 0);
}
	
void Step::setinitValue(double initValue, int index) {
	this->initValue[index] = initValue;
}

void Step::setStepHeight(double stepHeight) {
	this->setStepHeight(stepHeight, 0);
}

void Step::setStepHeight(double stepHeight, int index) {
	this->stepHeight[index] = stepHeight;
}

void Step::setDelayTime(double delayTime) {
	reset();
	this->stepTime = delayTime;
}
