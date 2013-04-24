/*
 * Output.hpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#ifndef OUTPUT_HPP_
#define OUTPUT_HPP_

#include "AnSignal.hpp"

class Output
{
public:
	Output(AnSignal* signal);
	virtual ~Output();

	virtual AnSignal* getSignal();

private:
	AnSignal* signal;
};

#endif /* OUTPUT_HPP_ */
