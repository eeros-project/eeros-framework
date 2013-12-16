#include "ExampleSafetyProperties.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>


using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

ExampleSafetyProperties::ExampleSafetyProperties() {

	HAL& hal = HAL::instance();
	
	// ############ Define criticcal outputs ############
	power = hal.getLogicSystemOutput("power");
	wd = hal.getLogicSystemOutput("wd");
	enable0 = hal.getLogicSystemOutput("enable0");
	enable1 = hal.getLogicSystemOutput("enable1");
	brake0 = hal.getLogicSystemOutput("brake0");
	brake1 = hal.getLogicSystemOutput("brake1");
	
	criticalOutputs = { power, enable0, enable1, brake0, brake1 };
	
	// ############ Define criticcal inputs ############
	emergencyStop = hal.getLogicSystemInput("emergencyStop");
	q0 = hal.getRealSystemInput("q0");
	q1 = hal.getRealSystemInput("q1");
	
	criticalInputs = { emergencyStop, q0, q1 };
	
	// ############ Define Levels ############
	levels = {
		{ off,               "Ausgeschalteter Zustand",                    },
		{ swShutingDown,     "Software herunterfahren",                    },
		{ swInitializing,    "Aufstarten der Software",                    },
		{ swInitialized,     "Software ist initialisiert",                 },
		{ controlStopping,   "Reglertask wird gestoppt",                   },
		{ controlStarting,   "Reglertask wird gestartet",                  },
		{ emergency,         "Notauszustand",                              },
		{ resetingEmergency, "Zurücksetzen des Notaus",                    },
		{ systemOn,          "System ist bereit, aber noch ohne Leistung", },
		{ poweringDown,      "Leistung ausschalten",                       },
		{ poweringUp,        "Leistung einschalten",                       },
		{ powerOn,           "Leistung ist ein, Motoren werden geregelt",  },
		{ motionStopping,    "",                                           },
		{ motionStarting,    "",                                           },
		{ moving,            "",                                           }
	};
	
	// ############ Add events to the levels ############
	level(off              ).addEvent(doSwInit,            swInitializing,    kPublicEvent);
	level(swShutingDown    ).addEvent(swShutDownDone,      off,               kPrivateEvent);
	level(swInitializing   ).addEvent(swInitDone,          swInitialized,     kPrivateEvent);
	level(swInitialized    ).addEvent(doOff,               swShutingDown,     kPublicEvent);
	level(swInitialized    ).addEvent(doControlStart,      controlStarting,   kPublicEvent);
	level(controlStopping  ).addEvent(controlStoppingDone, swInitialized,     kPrivateEvent);
	level(controlStarting  ).addEvent(controlStartingDone, systemOn,          kPrivateEvent);
	level(emergency        ).addEvent(doEmergencyReset,    resetingEmergency, kPublicEvent);
	level(resetingEmergency).addEvent(emergencyResetDone,  systemOn,          kPrivateEvent);
	level(systemOn         ).addEvent(doStopControl,       controlStopping,   kPublicEvent);
	level(systemOn         ).addEvent(doPoweringUp,        poweringUp,        kPublicEvent);
	level(poweringDown     ).addEvent(poweringDownDone,    systemOn,          kPrivateEvent);
	level(poweringUp       ).addEvent(poweringUpDone,      powerOn,           kPrivateEvent);
	level(powerOn          ).addEvent(doPoweringDown,      poweringDown,      kPublicEvent);
	level(powerOn          ).addEvent(doStartingMotion,    motionStarting,    kPublicEvent);
	level(motionStopping   ).addEvent(motionStoppingDone,  powerOn,           kPrivateEvent);
	level(motionStarting   ).addEvent(motionStartingDone,  moving,            kPrivateEvent);
	level(moving           ).addEvent(doMotionStopping,    motionStopping,    kPublicEvent);
	
	addEventToLevelAndAbove(powerOn, doEmergency, emergency, kPublicEvent);
	
	// ############ Define input states and events for all levels ############
	level(off              ).setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	level(swShutingDown    ).setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	level(swInitializing   ).setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	level(swInitialized    ).setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	level(controlStopping  ).setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	level(controlStarting  ).setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	level(emergency        ).setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	level(resetingEmergency).setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	level(systemOn         ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	level(poweringDown     ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	level(poweringUp       ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	level(powerOn          ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	level(motionStopping   ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	level(motionStarting   ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	level(moving           ).setInputActions({ check(emergencyStop, true, doEmergency), range(q0, 0.0, 6.283, doMotionStopping), range(q1, 0.0, 6.283, doMotionStopping) });
	
	// ############ Define output states and events for all levels ############
	level(off              ).setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), leave(wd) });
	level(swShutingDown    ).setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), leave(wd) });
	level(swInitializing   ).setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), leave(wd) });
	level(swInitialized    ).setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), leave(wd) });
	level(controlStopping  ).setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), leave(wd) });
	level(controlStarting  ).setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), leave(wd) });
	level(emergency        ).setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), leave(wd) });
	level(resetingEmergency).setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), leave(wd) });
	level(systemOn         ).setOutputActions({ set(power, true ), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ), toggle(wd) });
	level(poweringDown     ).setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, true ), set(brake1, true ), toggle(wd) });
	level(poweringUp       ).setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, true ), set(brake1, true ), toggle(wd) });
	level(powerOn          ).setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, false), set(brake1, false), toggle(wd) });
	level(motionStopping   ).setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, false), set(brake1, false), toggle(wd) });
	level(motionStarting   ).setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, false), set(brake1, false), toggle(wd) });
	level(moving           ).setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, false), set(brake1, false), toggle(wd) });
	
		// Define and add level functions
	level(swInitializing).setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(swInitDone);
	});
	
	// Define entry level
	entryLevel = off;
}

ExampleSafetyProperties::~ExampleSafetyProperties() {
	// nothing to do
}

