#ifndef ORG_EEROS_CONTROL_ANSIGNAL_HPP_
#define ORG_EEROS_CONTROL_ANSIGNAL_HPP_

#include <string>

#include <eeros/types.hpp>
#include <eeros/control/Signal.hpp>

struct anDatum {
	double value;
	uint64_t timestamp;
	std::string name;
	std::string unit;
	std::string coordinateSystem;
};

class AnSignal : public Signal {
public:
	AnSignal();
	AnSignal(std::string signalName, std::string unit, std::string coordinateSystem = "");
	AnSignal(std::string signalName[], std::string unit[], std::string coordinateSystem[], int length);
	virtual ~AnSignal();

	virtual std::string getLabel();
	virtual std::string getLabel(int index);
	virtual double getValue();
	virtual double getValue(int index);
	virtual uint64_t getTimestamp();
	virtual uint64_t getTimestamp(int index);	
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
};

#endif /* ORG_EEROS_CONTROL_ANSIGNAL_HPP_ */

