/*
 * Input.hpp
 *
 *  Created on: 11.04.2013
 *      Author: zueger1
 */

#ifndef INPUT_HPP_
#define INPUT_HPP_

#include "Signal.hpp"
#include "Output.hpp"

class Input {
public:
	Input();
	virtual ~Input();

	virtual Signal* getSignal();

	virtual bool connect(Output* output);
	virtual void disconnect();
	virtual bool isConnected();

private:
	Output* connectedOutput;
};

#endif /* INPUT_HPP_ */
