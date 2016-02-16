#ifndef EXAMPLESAFETYPROPERTIES_HPP
#define EXAMPLESAFETYPROPERTIES_HPP

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/PeripheralInput.hpp>

class ExampleSafetyProperties : public eeros::safety::SafetyProperties {
public:
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

    ExampleSafetyProperties();
    virtual ~ExampleSafetyProperties();
	
	// criticcal outputs
	eeros::hal::PeripheralOutput<bool>* power;
	eeros::hal::PeripheralOutput<bool>* wd;
	eeros::hal::PeripheralOutput<bool>* enable0;
	eeros::hal::PeripheralOutput<bool>* enable1;
	eeros::hal::PeripheralOutput<bool>* brake0;
	eeros::hal::PeripheralOutput<bool>* brake1;
	
	// criticcal inputs
	eeros::hal::PeripheralInput<bool>* emergencyStop;
	eeros::hal::PeripheralInput<double>* q0;
	eeros::hal::PeripheralInput<double>* q1;
};

#endif // EXAMPLESAFETYPROPERTIES_HPP
