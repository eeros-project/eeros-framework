/*
 * Step.hpp
 *
 *  Created on: 15.04.2013
 *      Author: zueger1
 */

#ifndef STEP_HPP_
#define STEP_HPP_

#include "AnSignal.hpp"
#include "Block1o.hpp"
#include "System.hpp"

class Step: public Block1o
{
public:
	Step();
	Step(AnSignal* sigal, double initValue = 0, double stepHeight = 1, double delayTime = 0);
	virtual ~Step();

	virtual void run();
	virtual void reset();

private:
	double initValue;
	double stepHeight;
	double stepTime;
	bool stepDone;
	bool first;
};

#endif /* STEP_HPP_ */
