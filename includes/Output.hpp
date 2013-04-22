/*
 * Output.hpp
 *
 *  Created on: 11.04.2013
 *      Author: zueger1
 */

#ifndef OUTPUT_HPP_
#define OUTPUT_HPP_

#include "Signal.hpp"

class Output
{
public:
	Output();
	virtual ~Output();

	virtual Signal* getSignal();

private:
	Signal* signal;
};

#endif /* OUTPUT_HPP_ */
