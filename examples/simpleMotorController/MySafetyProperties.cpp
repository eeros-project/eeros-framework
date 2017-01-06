#include "MySafetyProperties.hpp"
#include "MyControlSystem.hpp"

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

MySafetyProperties::MySafetyProperties(MyControlSystem& controlSys) : 
	controlSys(controlSys), 
	// ############ Define Levels ############
	off("Software is off"),
	emergencyState("Emergency state"),
	systemOn("System is ready, power off"),
	startingControl("System is starting controller"),
	stoppingControl("System is stopping controller"),
	powerOn("Power is on, motors are controlled"),
	moving("System is moving"),
	
	doSystemOn("Switch System on"),
	doSystemOff("Switch System off"),
	startControl("Start Control"),
	stopControl("Stop Control"),
	startControlDone("Control started"),
	stopControlDone("Control stopped"),
	startMoving("Start moving"),
	stopMoving("Stop moving"),
	doEmergency("Emergency"),
	resetEmergency("Reset emergency")
	
	{
	
	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	enable = hal.getLogicOutput("enable");
	
	criticalOutputs = { enable };
	
	// ############ Define critical inputs ############
	emergency = hal.getLogicInput("emergency");
	q = hal.getRealInput("q");
	
	criticalInputs = { emergency, q };
	
	off		.addEvent(doSystemOn,                     systemOn,                   kPublicEvent  );
	systemOn	.addEvent(startControl,                   startingControl,            kPublicEvent  );
	systemOn	.addEvent(doSystemOff,                    off,                        kPublicEvent  );
	startingControl	.addEvent(startControlDone,               powerOn,                    kPrivateEvent );
	stoppingControl	.addEvent(stopControlDone,                systemOn,                   kPrivateEvent );
	powerOn		.addEvent(startMoving,                    moving,                     kPublicEvent  );
	powerOn		.addEvent(stopControl,                    powerOn,                    kPublicEvent  );
	moving		.addEvent(stopMoving,                     powerOn,                    kPublicEvent  );
	emergencyState	.addEvent(resetEmergency,                 systemOn,                   kPublicEvent  );
	
	// Add events to multiple levels
	addEventToLevelAndAbove(systemOn, doEmergency, emergencyState, kPublicEvent);
		
	// ############ Define input states and events for all levels ############
	off		.setInputActions( { ignore(emergency) });
	emergencyState	.setInputActions( { ignore(emergency) });
	systemOn	.setInputActions( { check(emergency, true , doEmergency) });
	startingControl	.setInputActions( { check(emergency, true , doEmergency) });
	stoppingControl	.setInputActions( { check(emergency, true , doEmergency) });
	powerOn		.setInputActions( { check(emergency, true , doEmergency) });
	moving		.setInputActions( { check(emergency, true , doEmergency) });
	
	// Define and add level functions
	off.setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(doSystemOn);
	});
	
	systemOn.setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(startControl); // TODO read input
	});
	
	startingControl.setLevelAction([&](SafetyContext* privateContext) {
		controlSys.start();
		privateContext->triggerEvent(startControlDone);
	});
	
	stoppingControl.setLevelAction([&](SafetyContext* privateContext) {
		controlSys.stop();
		privateContext->triggerEvent(stopControlDone);
	});
	
	powerOn.setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(startMoving); // TODO read input
	});
	
	// Define entry level
	setEntryLevel(off);
}

MySafetyProperties::~MySafetyProperties() {
	// nothing to do
}
