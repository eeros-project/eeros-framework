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
	resetEmergency("Reset emergency"),
	abort("abort")
	
	{
	
	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	enable = hal.getLogicOutput("enable");
	
	criticalOutputs = { enable };
	
	// ############ Define critical inputs ############
	emergency = hal.getLogicInput("emergency");
	
	criticalInputs = { emergency };
	
	addLevel(off);
	addLevel(systemOn);
	addLevel(startingControl);
	addLevel(stoppingControl);
	addLevel(powerOn);
	addLevel(moving);
	addLevel(emergencyState);
	
	off		.addEvent(doSystemOn,                     systemOn,                   kPublicEvent  );
	systemOn	.addEvent(startControl,                   startingControl,            kPublicEvent  );
	systemOn	.addEvent(doSystemOff,                    off,                        kPublicEvent  );
	startingControl	.addEvent(startControlDone,               powerOn,                    kPrivateEvent );
	stoppingControl	.addEvent(stopControlDone,                off,                   kPrivateEvent );
	powerOn		.addEvent(startMoving,                    moving,                     kPublicEvent  );
	powerOn		.addEvent(stopControl,                    powerOn,                    kPublicEvent  );
	moving		.addEvent(stopMoving,                     powerOn,                    kPublicEvent  );
	emergencyState	.addEvent(resetEmergency,                 systemOn,                   kPublicEvent  );
	
	// Add events to multiple levels
	addEventToLevelAndAbove(systemOn, doEmergency, emergencyState, kPublicEvent);
	addEventToLevelAndAbove(startingControl, abort, stoppingControl, kPublicEvent);
		
	// ############ Define input states and events for all levels ############
	off		.setInputActions( { ignore(emergency) });
	emergencyState	.setInputActions( { ignore(emergency) });
	systemOn	.setInputActions( { check(emergency, true , doEmergency) });
	startingControl	.setInputActions( { check(emergency, true , doEmergency) });
	stoppingControl	.setInputActions( { check(emergency, true , doEmergency) });
	powerOn		.setInputActions( { check(emergency, true , doEmergency) });
	moving		.setInputActions( { check(emergency, true , doEmergency) });
	
	off		.setOutputActions( { set(enable, false) } );;
	emergencyState	.setOutputActions( { set(enable, false) } );;
	systemOn	.setOutputActions( { set(enable, false) } );;
	startingControl	.setOutputActions( { set(enable, false) } );;
	stoppingControl	.setOutputActions( { set(enable, false) } );;
	powerOn		.setOutputActions( { set(enable, false) } );;
	moving		.setOutputActions( { set(enable, true) } );;
	
	// Define and add level functions
	off.setLevelAction([&](SafetyContext* privateContext) {
		Executor::stop();
	});
	
	systemOn.setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(startControl); // TODO read input
	});
	
	startingControl.setLevelAction([&](SafetyContext* privateContext) {
		// TODO start controlSys if Executor allows this
		static int cnt = 0;
		cnt++;
		if(cnt > 500){	// wait 500ms
			privateContext->triggerEvent(startControlDone);
		}
	});
	
	stoppingControl.setLevelAction([&](SafetyContext* privateContext) {
		// TODO stop controlSys if Executor allows this
		privateContext->triggerEvent(stopControlDone);
	});
	
	powerOn.setLevelAction([&](SafetyContext* privateContext) {
// 		static int cnt = 0;
// 		cnt++;
// 		if(cnt > 100){
// 			cnt = 0;
// 			std::cout << "[" << "safety" << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue()<< std::endl;
// 		}
		privateContext->triggerEvent(startMoving); // TODO read input
	});
	
	moving.setLevelAction([&](SafetyContext* privateContext) {
		// moving
	});
	
	// Define entry level
	setEntryLevel(off);
	
	exitFunction = ([&](SafetyContext* privateContext){
		privateContext->triggerEvent(abort);
	});
	
}

MySafetyProperties::~MySafetyProperties() {
	// nothing to do
}
