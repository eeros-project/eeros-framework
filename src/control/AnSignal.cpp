/*
 * AnSignal.cpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#include "control/AnSignal.hpp"

AnSignal::AnSignal()
{
	this->length = 1;
	this->dat = new anDatum[1];
	this->dat[0].value = 0;
	this->dat[0].timestamp = 0;
	this->dat[0].name = "unnamed";
	this->dat[0].unit = "";
	this->dat[0].coordinateSystem = "";
}

AnSignal::AnSignal(std::string signalName, std::string unit, std::string coordinateSystem)
{
	this->length = 1;
	this->dat = new anDatum[1];
	this->dat[0].value = 0;
	this->dat[0].timestamp = 0;
	this->dat[0].name = signalName;
	this->dat[0].unit = unit;  // TODO check if given
	this->dat[0].coordinateSystem = coordinateSystem;  // TODO check if given
}

AnSignal::AnSignal(std::string signalName[], std::string unit[], std::string coordinateSystem[], int length)
{
    this->length = length;
	this->dat = new anDatum[length];
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

AnSignal::~AnSignal()
{
	delete dat;
}

double AnSignal::getValue()
{
    return this->dat[0].value;
}

double AnSignal::getValue(int index)
{
    if(index < this->length) return this->dat[index].value;
    return 0;
}

int AnSignal::getLength()
{
    return this->length;
}

std::string AnSignal::getName()
{
	return this->getName(0);
}

std::string AnSignal::getName(int index)
{
	return this->dat[index].name;
}

std::string AnSignal::getUnit()
{
	return this->getUnit(0);
}

std::string AnSignal::getUnit(int index)
{
	return this->dat[index].unit;
}

std::string AnSignal::getCoordinateSystem()
{
	return this->getCoordinateSystem(0);
}

std::string AnSignal::getCoordinateSystem(int index)
{
	return this->dat[index].coordinateSystem;
}

void AnSignal::setValue(double newValue)
{
	this->setValue(newValue, 0);
}

void AnSignal::setValue(double newValue, int index)
{
	this->dat[index].value = newValue;
}

void AnSignal::setValue(double newValue[])
{
	for(int i = 0; i < this->length; i++)
	{
		this->dat[i].value = newValue[i];
	}
}

bool AnSignal::isCompatible(AnSignal* signal)
{
	return false; // TODO implement this
}



