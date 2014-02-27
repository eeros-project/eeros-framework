#ifndef MYSAFETYPROPERTIES_HPP_
#define MYSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>

	// Name all levels
	enum {
		off = 0,
		emergencyState = 1,
		systemOn = 10,
		startingControl = 11,
		stoppingControl = 12,
		powerOn = 20,
		moving = 30
	};
	
	// Define all possible events
	enum {
		doSystemOn = 100,
		doSystemOff = 101,
		startControl = 102,
		stopControl = 103,
		startControlDone = 104,
		stopControlDone = 105,
		startMoving = 106,
		stopMoving = 107,
		doEmergency = 108,
		resetEmergency = 109
	};

class MySafetyProperties : public eeros::safety::SafetyProperties {
			
	public:
		MySafetyProperties();
		virtual ~MySafetyProperties();
	
		// critical outputs
		eeros::hal::SystemOutput<bool>* enable;
		
		// critical inputs
		eeros::hal::SystemInput<bool>* emergency;
		eeros::hal::SystemInput<double>* q;
};

#endif // MYSAFETYPROPERTIES_HPP_