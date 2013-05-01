/*
 * Output.cpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#include <eeros/control/Output.hpp>

Output::Output(AnSignal signal)
{
	this->signal = signal;

}

Output::~Output()
{
	// nothing to do
}

AnSignal* Output::getSignal()
{
	return &signal;
}

void Output::setSignal(AnSignal newSignal)
{
	this->signal = newSignal;
}