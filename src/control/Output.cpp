/*
 * Output.cpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#include "Output.hpp"

Output::Output(AnSignal* signal) {
	this->signal = signal;

}

Output::~Output() {
	delete this->signal;
}

AnSignal* Output::getSignal() {
	return signal;
}
