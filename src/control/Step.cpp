/*
 * Step.cpp
 *
 *  Created on: 15.04.2013
 *      Author: Martin Zueger
 */

#include <eeros/control/Step.hpp>


Step::Step(double initValue, double stepHeight, double delayTime)
{
	this->out = new Output(AnSignal());
	this->initValue = new double[1];
	this->stepHeight = new double[1];
	
	this->initValue[0] = initValue;
	this->stepHeight[0] = stepHeight;
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::Step(AnSignal signal, double initValue, double stepHeight, double delayTime)
{
	this->out = new Output(signal);
	this->initValue = new double[1];
	this->stepHeight = new double[1];
	
	this->initValue[0] = initValue;
	this->stepHeight[0] = stepHeight;
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::Step(AnSignal signal, double initValue[], double stepHeight[], double delayTime)
{
	int length = signal.getLength();
	this->out = new Output(signal);
	this->initValue = new double[length];
	this->stepHeight = new double[length];
	
	for(int i = 0; i < length; i++)
	{
		this->initValue[i] = initValue[i];
		this->stepHeight[i] = stepHeight[i];
	}
	this->stepTime = delayTime;
	this->stepDone = false;
	this->first = true;
}

Step::~Step()
{
	delete this->out;
	delete this->initValue;
	delete this->stepHeight;
}

void Step::run()
{
	if (first)
	{
		stepTime += System::getTime();
		first = false;
	}
	if (!stepDone && System::getTime() >= stepTime)
	{
		for(int i = 0; i < out->getSignal()->getLength(); i++)
		{
			out->getSignal()->setValue(initValue[i] + stepHeight[i], i);
		}
		stepDone = true;
	}
}

void Step::reset()
{
	stepDone = false;
	first = true;
}

void Step::setinitValue(double initValue)
{
	this->setinitValue(initValue, 0);
}
	
void Step::setinitValue(double initValue, int index)
{
	this->initValue[index] = initValue;
}

void Step::setStepHeight(double stepHeight)
{
	this->setStepHeight(stepHeight, 0);
}

void Step::setStepHeight(double stepHeight, int index)
{
	this->stepHeight[index] = stepHeight;
}

void Step::setDelayTime(double delayTime)
{
	reset();
	this->stepTime = delayTime;
}
