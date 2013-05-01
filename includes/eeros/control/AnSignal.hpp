/*
 * AnSignal.hpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_ANSIGNAL_HPP_
#define ORG_EEROS_CONTROL_ANSIGNAL_HPP_

#include <string>
#include <stdint.h>

struct anDatum {
	double value;
	uint32_t timestamp;
	std::string name;
	std::string unit;
	std::string coordinateSystem;
};

class AnSignal {
public:
	AnSignal();
	AnSignal(std::string signalName, std::string unit, std::string coordinateSystem = "");
	AnSignal(std::string signalName[], std::string unit[], std::string coordinateSystem[], int length);
	virtual ~AnSignal();

	virtual double getValue();
	virtual double getValue(int index);
	virtual int getLength();
	virtual std::string getName();
	virtual std::string getName(int index);
	virtual std::string getUnit();
	virtual std::string getUnit(int index);
	virtual std::string getCoordinateSystem();
	virtual std::string getCoordinateSystem(int index);
	
	virtual void setValue(double newValue);
	virtual void setValue(double newValue, int index);
	virtual void setValue(double newValue[]);
	
	virtual bool isCompatible(AnSignal* signal);

private:
	anDatum* dat;
	int length;
};

#endif /* ORG_EEROS_CONTROL_ANSIGNAL_HPP_ */

