#ifndef ORG_EEROS_CONTROL_STEP_HPP_
#define ORG_EEROS_CONTROL_STEP_HPP_

#include <vector>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>

class Step: public Block1o {
public:
	Step(double initValue = 0.0, double stepHeight = 1.0, double delayTime = 1.0, sigdim_t dim = 1);
	Step(const double initValue[], const double stepHeight[], const double delayTime, sigdim_t dim);
	Step(const std::vector<double> initValues, const std::vector<double> stepHeight, double delayTime, sigdim_t dim);
	virtual ~Step();

	virtual void run();
	
	virtual void reset();
	virtual void setInitValue(double initValue);
	virtual void setInitValue(sigindex_t index, double initValue);
	virtual void setStepHeight(double stepHeight);
	virtual void setStepHeight(sigindex_t index, double stepHeight);
	virtual void setDelayTime(double delayTime);

protected:
	std::vector<double> initValue;
	std::vector<double> stepHeight;
	double stepTime;
	bool stepDone;
	bool first;
};

#endif /* ORG_EEROS_CONTROL_STEP_HPP_ */
