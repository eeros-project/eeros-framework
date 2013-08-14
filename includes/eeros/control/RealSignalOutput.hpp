#ifndef ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_
#define ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_

#include <string>

#include <eeros/types.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Signal.hpp>

enum { kSignalTypeReal = 42 };

struct realSignalDatum {
	double value;
	uint64_t timestamp;
	std::string name;
	std::string unit;
	std::string coordinateSystem;
	std::string sendingDirection; // TODO remove this from realSignalDatum
};

class RealSignalOutput : virtual public Signal, virtual public Output {
public:
	static const std::string SENDING_DIRECTION_SERVER_TO_CLIENT;
	static const std::string SENDING_DIRECTION_CLIENT_TO_SERVER;

	RealSignalOutput(sigdim_t dim = 1, std::string sendingDirection = SENDING_DIRECTION_SERVER_TO_CLIENT);
	virtual ~RealSignalOutput();

	virtual sigtype_t getType() const;
	virtual std::string getLabel() const;
	virtual std::string getLabel(sigindex_t index) const;
	virtual double getValue() const;
	virtual double getValue(sigindex_t index) const;
	virtual uint64_t getTimestamp() const;
	virtual uint64_t getTimestamp(sigindex_t index) const;
	virtual std::string getName() const;
	virtual std::string getName(sigindex_t index) const;
	virtual std::string getUnit() const;
	virtual std::string getUnit(sigindex_t index) const;
	virtual std::string getCoordinateSystem() const;
	virtual std::string getCoordinateSystem(sigindex_t index) const;
	virtual std::string getSendingDirection() const;
	virtual std::string getSendingDirection(sigindex_t index) const;
	
	virtual void setValue(double newValue);
	virtual void setValue(double newValue, sigindex_t index);
	virtual void setValue(double newValue[]);
	virtual void setTimeStamp(uint64_t timestamp);
	virtual void setTimeStamp(uint64_t timestamp, sigindex_t index);
	virtual void setName(std::string signalName);
	virtual void setName(std::string signalName, sigindex_t index);
	virtual void setUnit(std::string unit);
	virtual void setUnit(std::string unit, sigindex_t index);
	virtual void setCoordinateSystem(std::string coordinateSystem);
	virtual void setCoordinateSystem(std::string coordinateSystem, sigindex_t index);
	virtual void setSendingDirection(std::string sendingDirection);
	virtual void setSendingDirection(std::string sendingDirection, sigindex_t index);

private:
	realSignalDatum* dat;
};

#endif /* ORG_EEROS_CONTROL_REALSIGNALOUTPUT_HPP_ */

