/*
 * Output.hpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_OUTPUT_HPP_
#define ORG_EEROS_CONTROL_OUTPUT_HPP_

#include "control/AnSignal.hpp"

class Output
{
public:
	Output(AnSignal signal);
	virtual ~Output();

	virtual AnSignal* getSignal();
	virtual void setSignal(AnSignal newSignal);

private:
	AnSignal signal;
};

#endif /* ORG_EEROS_CONTROL_OUTPUT_HPP_ */
