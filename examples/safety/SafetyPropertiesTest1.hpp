#ifndef SAFETYPROPERTIESTEST1_HPP
#define SAFETYPROPERTIESTEST1_HPP

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/PeripheralInput.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/Executor.hpp>
	
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

extern volatile bool running;

class SafetyPropertiesTest1 : public SafetyProperties {
public:
	SafetyPropertiesTest1()  : 
		seStartInitializing("start initializing"),
		seInitializingDone("initialization done"),
		seStartRunning("start running"),
		seShutDown("start shutting down"),
		seStopRunning("stop running"),
		seSwitchingOff("switching off"),
		slOff("off"),
		slShuttingDown("shutting down"),
		slIinitializing("initializing"),
		slInitialized("initialized"),
		slRunning("running")
		{	

		HAL& hal = HAL::instance();

		// ############ Define critical outputs ############
		out1 = hal.getLogicPeripheralOutput("out1");
		criticalOutputs = { out1 };

		// ############ Define critical inputs ############
		in1 = hal.getLogicPeripheralInput("in1");
		criticalInputs = { in1 };
		
		// ############ Add levels ############
		addLevel(slOff);
		addLevel(slShuttingDown);
		addLevel(slIinitializing);
		addLevel(slInitialized);
		addLevel(slRunning);
		
		// ############ Add events to the levels ############
		slOff.addEvent(seStartInitializing, slIinitializing, kPublicEvent);
		slShuttingDown.addEvent(seSwitchingOff, slOff, kPrivateEvent);
		slIinitializing.addEvent(seInitializingDone, slInitialized, kPublicEvent);
		slInitialized.addEvent(seStartRunning, slRunning, kPublicEvent);
		slRunning.addEvent(seStopRunning, slInitialized, kPrivateEvent);
		addEventToLevelAndAbove(slIinitializing, seShutDown, slShuttingDown, kPublicEvent);

		// ############ Define input states and events for all levels ############
		slOff.setInputActions({ ignore(in1) });
		slShuttingDown.setInputActions({ ignore(in1) });
		slIinitializing.setInputActions({ ignore(in1) });
		slInitialized.setInputActions({ check(in1, true, seStartRunning) });
		slRunning.setInputActions({ check(in1, false, seStopRunning) });

		// ############ Define output states and events for all levels ############
		slOff.setOutputActions({ set(out1, false) });
		slShuttingDown.setOutputActions({ set(out1, false) });
		slIinitializing.setOutputActions({ set(out1, false) });
		slInitialized.setOutputActions({ set(out1, false) });
		slRunning.setOutputActions({ set(out1, true) });

		// Define and add level functions
		slOff.setLevelAction([&](SafetyContext* privateContext) {Executor::stop();});
		slShuttingDown.setLevelAction([&](SafetyContext* privateContext) {privateContext->triggerEvent(seSwitchingOff);});

		// Define entry level
		setEntryLevel(slOff);
		
		// Define action when exiting application with Ctrl-C 
		exitFunction = [&](SafetyContext* privateContext) {privateContext->triggerEvent(seShutDown);};

	}
	
	// critical outputs
	PeripheralOutput<bool>* out1;
	
	// critical inputs
	PeripheralInput<bool>* in1;
	
	SafetyEvent seStartInitializing;
	SafetyEvent seInitializingDone;
	SafetyEvent seStartRunning;
	SafetyEvent seShutDown;
	SafetyEvent seStopRunning;
	SafetyEvent seSwitchingOff;
	
	SafetyLevel slOff;
	SafetyLevel slShuttingDown;
	SafetyLevel slIinitializing;
	SafetyLevel slInitialized;
	SafetyLevel slRunning;
};

#endif // SAFETYPROPERTIESTEST1_HPP
