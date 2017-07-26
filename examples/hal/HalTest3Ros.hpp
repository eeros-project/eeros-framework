#ifndef ORG_EEROS_HALTEST3ROS_HPP_
#define ORG_EEROS_HALTEST3ROS_HPP_

#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <unistd.h>

using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::hal;


// thsi block prints value of signal
template < typename T = double >
class Print : public eeros::control::Block1i<T> {
	public:
		Print(int modulo=1) : modulo(modulo), counter(0) { }
		virtual void run() {
			if ( (counter % modulo) == 0 ) {
				std::cout << this->in.getSignal().getValue() << std::endl;
			}
			counter++;
		}
		int modulo;
		uint64_t counter;
};


class MyControlSystem {
public:
	MyControlSystem(double ts):
		dt(ts),
		printDouble0(1),
		printBool0(1),
		analogIn0("scanTimeIn0"),		// argument has to match signalId of json
		digitalIn0("batteryPresent0"),
		analogOut0("scanTimeEchoOut0"),
		digitalOut0("batteryPresentEchoOut0"),
		timedomain("Main time domain", dt, true) 
		{
		
		// Connect Blocks
		printDouble0.getIn().connect(analogIn0.getOut());
		printBool0.getIn().connect(digitalIn0.getOut());
		analogOut0.getIn().connect(analogIn0.getOut());
		digitalOut0.getIn().connect(digitalIn0.getOut());
		
		// Run blocks
		timedomain.addBlock(&analogIn0);
		timedomain.addBlock(&digitalIn0);
		timedomain.addBlock(&printBool0);
		timedomain.addBlock(&printDouble0);
		timedomain.addBlock(&analogOut0);
		timedomain.addBlock(&digitalOut0);
		
		eeros::Executor::instance().add(timedomain);
	}
	virtual ~MyControlSystem() { }
	
	Print<double> printDouble0;
	Print<bool> printBool0;
	PeripheralInput<double>		analogIn0;
	PeripheralInput<bool>		digitalIn0;
	PeripheralOutput<double>	analogOut0;
	PeripheralOutput<bool>		digitalOut0;
	double dt;
	bool realtime;
	eeros::control::TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
public:
	MySafetyProperties() : slOff("off") {	
		addLevel(slOff);
		setEntryLevel(slOff);
	}
	
	SafetyLevel slOff;
};


#endif // ORG_EEROS_HALTEST3ROS_HPP_