/*
 * Input.cpp
 *
 *  Created on: 11.04.2013
 *      Author: zueger1
 */

#include "Input.hpp"

Input::Input()
{
	// TODO Auto-generated constructor stub
}

Input::~Input()
{
	// TODO Auto-generated destructor stub
}

Signal* Input::getSignal()
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
