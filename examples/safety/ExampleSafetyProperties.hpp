#ifndef EXAMPLESAFETYPROPERTIES_HPP
#define EXAMPLESAFETYPROPERTIES_HPP

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/SystemOutput.hpp>
#include <eeros/hal/SystemInput.hpp>

class ExampleSafetyProperties : public eeros::safety::SafetyProperties {

	// Define all possible events
	enum {
		doSwInit,
		swShutDownDone,
		swInitDone,
		doOff,
		doControlStart,
		controlStoppingDone,
		controlStartingDone,
		doEmergencyReset,
		emergencyResetDone,
		doStopControl,
		doPoweringUp,
		poweringDownDone,
		poweringUpDone,
		doPoweringDown,
		doStartingMotion,
		motionStoppingDone,
		motionStartingDone,
		doMotionStopping,
		doEmergency
	};
	
	// Name all levels
	enum {
		off = 10,
		swShutingDown = 11,
		swInitializing = 12,
		swInitialized = 20,
		controlStopping = 21,
		controlStarting = 22,
		emergency = 28,
		resetingEmergency = 29,
		systemOn = 30,
		poweringDown = 31,
		poweringUp = 32,
		powerOn = 40,
		motionStopping = 41,
		motionStarting = 42,
		moving = 50
	};
	
public:
    ExampleSafetyProperties();
    virtual ~ExampleSafetyProperties();
	
	// criticcal outputs
	eeros::hal::SystemOutput<bool>* power;
	eeros::hal::SystemOutput<bool>* wd;
	eeros::hal::SystemOutput<bool>* enable0;
	eeros::hal::SystemOutput<bool>* enable1;
	eeros::hal::SystemOutput<bool>* brake0;
	eeros::hal::SystemOutput<bool>* brake1;
	
	// criticcal inputs
	eeros::hal::SystemInput<bool>* emergencyStop;
	eeros::hal::SystemInput<double>* q0;
	eeros::hal::SystemInput<double>* q1;
};

#endif // EXAMPLESAFETYPROPERTIES_HPP
