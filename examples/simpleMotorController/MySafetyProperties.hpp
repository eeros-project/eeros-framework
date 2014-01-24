#ifndef MYSAFETYPROPERTIES_HPP_
#define MYSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>

// namespace scara {
class MySafetyProperties : public eeros::safety::SafetyProperties {

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
		doSystemOn,
		doSystemOff,
		startControl,
		stopControl,
		startControlDone,
		stopControlDone,
		startMoving,
		stopMoving,
		doEmergency,
		resetEmergency
	};
			
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