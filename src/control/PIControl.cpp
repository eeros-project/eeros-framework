#include <float.h>
#include <eeros/core/System.hpp>
#include <eeros/control/PIControl.hpp>

PIControl::PIControl(double kp, double ki) : Block2i1o(1), firstRun(true) {
	kps = new double[1];
	kps[0] = kp;
	kis = new double[1];
	kis[0] = ki;
	integrals = new double[1];
	integrals[0] = 0.0;
	antiWindupLowerLimits = new double[1];
	antiWindupLowerLimits[0] = DBL_MIN;
	antiWindupUpperLimits = new double[1];
	antiWindupUpperLimits[0] = DBL_MAX;
}

PIControl::PIControl(sigdim_t dim, double kps[], double kis[]) : Block2i1o(dim), firstRun(true) {
	this->kps = new double[dim];
	this->kis = new double[dim];
	integrals = new double[dim];
	antiWindupLowerLimits = new double[dim];
	antiWindupUpperLimits = new double[dim];
	for(int i = 0; i < dim; i++) {
		this->kps[i] = kps[i];
		this->kis[i] = kis[i];
		integrals[i] = 0.0;
		antiWindupLowerLimits[i] = DBL_MIN;
		antiWindupUpperLimits[i] = DBL_MAX;
	}
}

PIControl::~PIControl() {
 	delete [] kps;
 	delete [] kis;
 	delete [] integrals;
 	delete [] antiWindupLowerLimits;
	delete [] antiWindupUpperLimits;
}

void PIControl::setAntiWindupLimits(double lowerLimit, double upperLimit) {
	antiWindupLowerLimits[0] = lowerLimit;
	antiWindupUpperLimits[0] = upperLimit;
}

void PIControl::setAntiWindupLimits(double lowerLimits[], double upperLimits[]) {
	for (uint32_t i = 0; i < out.getLength(); i++) {
		antiWindupLowerLimits[i] = lowerLimits[i];
		antiWindupUpperLimits[i] = upperLimits[i];
	}
}

void PIControl::run() {
    uint64_t time = System::getTime();
    if (firstRun) {
    	previousTime = time;
    	firstRun = false;
    }
	for(int i = 0; i < out.getLength(); i++) {
        double ep = in2.getValue(i) - in1.getValue(i);
        integrals[i] += ep * ((time - previousTime) * 1e-12);
        double ei = kis[i] * integrals[i];

        if(ei < -antiWindupLowerLimits[i]) {
        	ei = -antiWindupLowerLimits[i];
        } else if(ei > antiWindupUpperLimits[i]) {
        	ei = antiWindupUpperLimits[i];
        }
        out.setValue((ep * kps[i] + ei), i);
	}
	out.setTimeStamp(time);
	previousTime = time;
}
