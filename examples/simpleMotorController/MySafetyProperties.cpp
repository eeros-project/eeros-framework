#include "MySafetyProperties.hpp"
#include "MyControlSystem.hpp"

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/core/Executor.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

MySafetyProperties::MySafetyProperties(MyControlSystem& controlSys, double dt) : 
	controlSys(controlSys),
	// ############ Define Levels ############
	slOff("Software is off"),
	slEmergency("Emergency state"),
	slSystemOn("System is ready, power off"),
	slStartingControl("System is starting control system"),
	slStoppingControl("System is stopping control system"),
	slPowerOn("Power is on, motors are controlled"),
	slMoving("System is moving"),
	
	doSystemOn("Switch System on"),
	doSystemOff("Switch System off"),
	startControl("Start Control"),
	stopControl("Stop Control"),
	startControlDone("Control started"),
	stopControlDone("Control stopped"),
	startMoving("Start moving"),
	stopMoving("Stop moving"),
	doEmergency("Emergency"),
	resetEmergency("Reset emergency"),
	abort("abort")
	{
	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	enable = hal.getLogicOutput("enable");
	
	criticalOutputs = { enable };
	
	// ############ Define critical inputs ############
	emergency = hal.getLogicInput("emergency");
	ready = hal.getLogicInput("readySig1");
	
	criticalInputs = { emergency, ready };
	
	addLevel(slOff);
	addLevel(slEmergency);
	addLevel(slSystemOn);
	addLevel(slStartingControl);
	addLevel(slStoppingControl);
	addLevel(slPowerOn);
	addLevel(slMoving);
	
	slOff			.addEvent(doSystemOn,		slSystemOn,		kPublicEvent  );
	slEmergency		.addEvent(resetEmergency,	slSystemOn,		kPublicEvent  );
	slSystemOn		.addEvent(startControl,		slStartingControl,	kPublicEvent  );
	slSystemOn		.addEvent(doSystemOff,		slOff,			kPublicEvent  );
	slStartingControl	.addEvent(startControlDone,	slPowerOn,		kPrivateEvent );
	slStoppingControl	.addEvent(stopControlDone,	slOff,			kPrivateEvent );
	slPowerOn		.addEvent(startMoving,		slMoving,		kPublicEvent  );
	slMoving		.addEvent(stopMoving,		slPowerOn,		kPublicEvent  );
	
	// Add events to multiple levels
	addEventToLevelAndAbove(slSystemOn, doEmergency, slEmergency, kPublicEvent);
	addEventToLevelAndAbove(slEmergency, abort, slStoppingControl, kPublicEvent);
		
	// ############ Define input states and events for all levels ############
	slOff			.setInputActions( { ignore(emergency), ignore(ready) });
	slEmergency		.setInputActions( { ignore(emergency), ignore(ready) });
	slSystemOn		.setInputActions( { check(emergency, true , doEmergency), ignore(ready) });
	slStartingControl	.setInputActions( { check(emergency, true , doEmergency), ignore(ready)});
	slStoppingControl	.setInputActions( { check(emergency, true , doEmergency), ignore(ready) });
	slPowerOn		.setInputActions( { check(emergency, true , doEmergency), ignore(ready) });
	slMoving		.setInputActions( { check(emergency, true , doEmergency), check(ready, true, doEmergency) });
	
	slOff			.setOutputActions( { set(enable, false) } );;
	slEmergency		.setOutputActions( { set(enable, false) } );;
	slSystemOn		.setOutputActions( { set(enable, false) } );;
	slStartingControl	.setOutputActions( { set(enable, false) } );;
	slStoppingControl	.setOutputActions( { set(enable, false) } );;
	slPowerOn		.setOutputActions( { set(enable, true) } );;
	slMoving		.setOutputActions( { set(enable, true) } );;
	
	// Define and add level functions
	slOff.setLevelAction([&](SafetyContext* privateContext) {
		Executor::stop();
	});
	
	slSystemOn.setLevelAction([&](SafetyContext* privateContext) {
		controlSys.timedomain.stop();
		// you may want to check here for a user input
		privateContext->triggerEvent(startControl); 
	});
	
	slStartingControl.setLevelAction([&,dt](SafetyContext* privateContext) {
		controlSys.timedomain.start();
		if(slStartingControl.getNofActivations() * dt > 2){	// wait 2s
			privateContext->triggerEvent(startControlDone);
		}
	});
	
	slStoppingControl.setLevelAction([&](SafetyContext* privateContext) {
		controlSys.timedomain.stop();
		privateContext->triggerEvent(stopControlDone);
	});
	
	slPowerOn.setLevelAction([&](SafetyContext* privateContext) {
		if(ready->get()){	// check if drive is ready
			privateContext->triggerEvent(startMoving);
		}
	});
		
	// Define entry level
	setEntryLevel(slOff);
	
	exitFunction = ([&](SafetyContext* privateContext){
		privateContext->triggerEvent(abort);
	});
	
}

MySafetyProperties::~MySafetyProperties() { }
