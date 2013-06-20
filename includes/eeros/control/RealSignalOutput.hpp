#ifndef ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_
#define ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_

#include <string>

#include <eeros/types.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Signal.hpp>

struct anDatum {
	double value;
	uint64_t timestamp;
	std::string name;
	std::string unit;
	std::string coordinateSystem;
};

class RealSignalOutput : virtual public Signal, virtual public Output {
public:
	RealSignalOutput(sigdim_t dim = 1);
//	RealSignalOutput(std::string signalName, std::string unit, std::string coordinateSystem = "");
//	RealSignalOutput(std::string signalName[], std::string unit[], std::string coordinateSystem[], int length);
	virtual ~RealSignalOutput();

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
	// TODO setter ergänzen
	virtual void setValue(double newValue);
	virtual void setValue(double newValue, int index);
	virtual void setValue(double newValue[]);

private:
	anDatum* dat;
};

#endif /* ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_ */

