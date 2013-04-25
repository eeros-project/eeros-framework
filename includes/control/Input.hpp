/*
 * Input.hpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_INPUT_HPP_
#define ORG_EEROS_CONTROL_INPUT_HPP_

#include "control/AnSignal.hpp"
#include "control/Output.hpp"

class Input {
public:
	Input();
	virtual ~Input();

	virtual AnSignal* getSignal();

	virtual bool connect(Output* output);
	virtual void disconnect();
	virtual bool isConnected();

private:
	Output* connectedOutput;
};

#endif /* ORG_EEROS_CONTROL_INPUT_HPP_ */
