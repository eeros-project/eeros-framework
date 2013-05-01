/*
 * Gain.hpp
 *
 *  Created on: 01.05.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_GAIN_HPP_
#define ORG_EEROS_CONTROL_GAIN_HPP_

#include <eeros/control/AnSignal.hpp>
#include <eeros/control/Block1i1o.hpp>

class Gain: public Block1i1o
{
public:
	Gain();
	Gain(AnSignal sigal, double gain = 1);
	Gain(AnSignal sigal, double gain[]);
	virtual ~Gain();

	virtual void run();
	
	virtual void enable();
    virtual void disable();
	virtual void setGain(double gain);
	virtual void setGain(double gain, int index);

private:
	double* gain;
	bool enabled;
};

#endif /* ORG_EEROS_CONTROL_GAIN_HPP_ */
