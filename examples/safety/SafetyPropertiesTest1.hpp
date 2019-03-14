#ifndef SAFETYPROPERTIESTEST1_HPP
#define SAFETYPROPERTIESTEST1_HPP

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/core/Executor.hpp>
	
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

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
		out = hal.getLogicOutput("out");
		criticalOutputs = { out };

		// ############ Define critical inputs ############
		in = hal.getLogicInput("in");
		criticalInputs = { in };
		
		// ############ Get test in and outputs ############ 
		outTest = hal.getLogicOutput("outTest");
		outTest->set(false);
		inTest = hal.getLogicInput("inTest");
		
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
		slOff.setInputActions({ ignore(in) });
		slShuttingDown.setInputActions({ ignore(in) });
		slIinitializing.setInputActions({ ignore(in) });
		slInitialized.setInputActions({ check(in, false, seStartRunning) });
		slRunning.setInputActions({ check(in, true, seStopRunning) });

		// ############ Define output states and events for all levels ############
		slOff.setOutputActions({ set(out, false) });
		slShuttingDown.setOutputActions({ set(out, false) });
		slIinitializing.setOutputActions({ set(out, false) });
		slInitialized.setOutputActions({ set(out, false) });
		slRunning.setOutputActions({ toggle(out) });

		// Define and add level functions
		slOff.setLevelAction([&](SafetyContext* privateContext) {Executor::stop();});
		slShuttingDown.setLevelAction([&](SafetyContext* privateContext) {privateContext->triggerEvent(seSwitchingOff);});
		slInitialized.setLevelAction([&](SafetyContext* privateContext) {
			if(slInitialized.getNofActivations() > 5){
				outTest->set(true);	// switch on in1 via sim-eeros
			}
		});
		slRunning.setLevelAction([&](SafetyContext* privateContext){
			if(slRunning.getNofActivations() > 5){
				outTest->set(false);	// switch off in1 via sim-eeros
			}
		});
		// Define entry level
		setEntryLevel(slOff);
		
		// Define action when exiting application with Ctrl-C 
		exitFunction = [&](SafetyContext* privateContext) {privateContext->triggerEvent(seShutDown);};

	}
	
	// critical outputs
	Output<bool>* out;
	
	// critical inputs
	Input<bool>* in;
	
	// test inputs
	Input<bool>* inTest;
	// test outputs
	Output<bool>* outTest;
	
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
