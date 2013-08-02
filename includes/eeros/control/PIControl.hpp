#ifndef ORG_EEROS_CONTROL_PICONTROL_HPP_
#define ORG_EEROS_CONTROL_PICONTROL_HPP_

#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/Block2i1o.hpp>

class PIControl: public Block2i1o {
public:
	PIControl(double kp, double ki);
	PIControl(sigdim_t dim, double kp[], double ki[]);
	virtual ~PIControl();

	void setAntiWindupLimits(double lowerLimit, double upperLimit);
	void setAntiWindupLimits(double lowerLimits[], double upperLimits[]);
	virtual void run();
private:
	bool firstRun;
	uint64_t previousTime;
	double *kps;
	double *kis;
	double *integrals;
	double *antiWindupLowerLimits;
	double *antiWindupUpperLimits;
};

#endif /* ORG_EEROS_CONTROL_PICONTROL_HPP_ */
