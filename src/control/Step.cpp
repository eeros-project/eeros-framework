/*
 * Step.cpp
 *
 *  Created on: 15.04.2013
 *      Author: zueger1
 */

#include "Step.hpp"


Step::Step()
{
	// TODO Auto-generated constructor stub

}

Step::Step(AnSignal* signal, double initValue, double stepHeight, double delayTime)
{
	this->out = new Output(signal);
	this->initValue = initValue;
	this->stepHeight = stepHeight;
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::~Step()
{
	// TODO Auto-generated destructor stub
}

void Step::run()
{
	if (first)
	{
		stepTime = System::getTime();
		first = false;
	}
	if (!stepDone && System::getTime() >= stepTime)
	{
		out->getSignal()->setValue(initValue + stepHeight);
		stepDone = true;
	}
}

void Step::reset()
{
	stepDone = false;
	first = true;
}
