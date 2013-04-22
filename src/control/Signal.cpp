/*
 * Signal.cpp
 *
 *  Created on: 11.04.2013
 *      Author: zueger1
 */

#include "Signal.hpp"

Signal::Signal()
{
    length = 1;
	dat = new datum[1];
	dat[0].value = 0;
	dat[0].timestamp = 0;
	dat[0].name = "";
	dat[0].unit = "";
	dat[0].coordinateSystem = "";
}

Signal::Signal(std::string signalName, std::string unit, std::string coordinateSystem)
{

}

Signal::Signal(std::string signalName[], std::string unit[], std::string coordinateSystem[], int length)
{
    this->length = length;
	dat = new datum[length];
	for (int i = 0; i < length; i++)
	{
		dat[i].value = 0;
		dat[i].timestamp = 0;
//		if (signalName[i] != NULL)
			dat[i].name = signalName[i];
//		else
//			dat[i].name = "";
//		if (unit[i] != NULL)
			dat[i].unit = unit[i];
//		else
//			dat[i].unit = "";
//		if (coordinateSystem[i] != NULL)
			dat[i].coordinateSystem = coordinateSystem[i];
//		else
//			dat[i].coordinateSystem = "";
	}
}

void Signal::setValue(double newValue) {

}

void Signal::setValue(double newValue[]) {

}

double Signal::getValue() {
    return dat[0].value;
}

double Signal::getValue(int index) {
    if(index < length) return dat[index].value;
    return 0;
}

int Signal::getLength() {
    return length;
}

Signal::~Signal()
{
	delete(dat);
}

