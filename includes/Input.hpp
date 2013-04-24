/*
 * Input.hpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#ifndef INPUT_HPP_
#define INPUT_HPP_

#include "AnSignal.hpp"
#include "Output.hpp"

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

#endif /* INPUT_HPP_ */
