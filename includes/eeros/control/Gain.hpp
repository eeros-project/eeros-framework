#ifndef ORG_EEROS_CONTROL_GAIN_HPP_
#define ORG_EEROS_CONTROL_GAIN_HPP_

#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/Block1i1o.hpp>

class Gain: public Block1i1o {
public:
	Gain(sigdim_t dim = 1);
	Gain(sigdim_t dim, double gain[]);
	virtual ~Gain();

	virtual void run();
	
	virtual void enable();
    virtual void disable();
	virtual void setGain(double gain, sigindex_t index = 0);

private:
	double* gain;
	bool enabled;
};

#endif /* ORG_EEROS_CONTROL_GAIN_HPP_ */
