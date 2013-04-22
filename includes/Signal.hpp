#ifndef ORG_EEROS_CONTROL_SIGNAL_HPP_
#define ORG_EEROS_CONTROL_SIGNAL_HPP_

#include <string>
#include <stdint.h>

struct datum {
	double value;
	uint32_t timestamp;
	std::string name;
	std::string unit;
	std::string coordinateSystem;
};

class Signal {
public:
	Signal();
	Signal(std::string signalName = "", std::string unit = "", std::string coordinateSystem = "");
	Signal(std::string signalName[], std::string unit[], std::string coordinateSystem[], int length = 1);
	virtual ~Signal();

	virtual double getValue();
	virtual double getValue(int index);
	virtual int getLength();
	virtual void setValue(double newValue);
	virtual void setValue(double newValue[]);

private:
	datum* dat;
};

#endif /* ORG_EEROS_CONTROL_SIGNAL_HPP_ */
