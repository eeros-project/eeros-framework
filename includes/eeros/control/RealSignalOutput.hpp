#ifndef ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_
#define ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_

#include <string>

#include <eeros/types.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Signal.hpp>

enum { kSignalTypeReal = 42 };

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
	virtual ~RealSignalOutput();

	virtual sigtype_t getType() const;
	virtual std::string getLabel() const;
	virtual std::string getLabel(int index) const;
	virtual double getValue() const;
	virtual double getValue(int index) const;
	virtual uint64_t getTimestamp() const;
	virtual uint64_t getTimestamp(int index) const;
	virtual std::string getName() const;
	virtual std::string getName(int index) const;
	virtual std::string getUnit() const;
	virtual std::string getUnit(int index) const;
	virtual std::string getCoordinateSystem() const;
	virtual std::string getCoordinateSystem(int index) const;
	
	virtual void setValue(double newValue);
	virtual void setValue(double newValue, int index);
	virtual void setValue(double newValue[]);
	virtual void setTimeStamp(uint64_t timestamp);
	virtual void setTimeStamp(uint64_t timestamp, int index);
	virtual void setName(std::string signalName);
	virtual void setName(std::string signalName, int index);
	virtual void setUnit(std::string unit);
	virtual void setUnit(std::string unit, int index);
	virtual void setCoordinateSystem(std::string coordinateSystem);
	virtual void setCoordinateSystem(std::string coordinateSystem, int index);

private:
	anDatum* dat;
};

#endif /* ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_ */

