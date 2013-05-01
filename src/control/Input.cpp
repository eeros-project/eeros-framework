/*
 * Input.cpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#include <eeros/control/Input.hpp>

Input::Input()
{
	// nothing to do
}

Input::~Input()
{
	// nothing to do
}

AnSignal* Input::getSignal()
{
	if (this->isConnected())
	{
		return this->connectedOutput->getSignal();
	}
	return NULL;
}

bool Input::connect(Output* output)
{
	if (output != NULL)
	{
		connectedOutput = output;
		return true;
	}
	return false;
}

void Input::disconnect()
{
	connectedOutput = NULL;
}

bool Input::isConnected()
{
	return connectedOutput != NULL;
}
