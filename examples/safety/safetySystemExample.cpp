#include <iostream>
#include <vector>
#include <initializer_list>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/hal/HAL.hpp>

int main() {
	std::cout << "Safety System Example started..." << std::endl;

	// Get Safety System instance
	SafetySystem& safetySys = SafetySystem::instance();
	
	// Get HAL instance
	HAL& hal = HAL::instance();
	
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
	
	// Define criticcal outputs
	SystemOutput<bool> power { hal.getLogicSystemOutput("power") };
	SystemOutput<bool> enable[] {
		hal.getLogicSystemOutput("enable0"),
		hal.getLogicSystemOutput("enable1"),
		hal.getLogicSystemOutput("enable2"),
		hal.getLogicSystemOutput("enable3")
	};
	SystemOutput<bool> brake[] {
		hal.getLogicSystemOutput("brake0"),
		hal.getLogicSystemOutput("brake1"),
		hal.getLogicSystemOutput("brake2"),
		hal.getLogicSystemOutput("brake3")
	};
// 	safetySys.defineCriticalOutputs({ // TODO
// 		power,
// 		enable[0],
// 		enable[1],
// 		enable[2],
// 		enable[3],
// 		brake[0],
// 		brake[1],
// 		brake[2],
// 		brake[3]
// 	});
	
	// Define criticcal inputs
	SystemInput<bool> emergencyStop { hal.getLogicSystemInput("emergencyStop") };
	SystemInput<double> q[] {
		hal.getRealSystemInput("q0"),
		hal.getRealSystemInput("q1"),
		hal.getRealSystemInput("q2"),
		hal.getRealSystemInput("q3")
	};
// 	safetySys.defineCriticalInputs({ // TODO
// 		emergencyStop,
// 		q[0],
// 		q[1],
// 		q[2],
// 		q[3]
// 	});
		
	// Define Levels
	safetySys.defineSafetyLevels({
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
	});
	
	// Add events to each level
	safetySys[off              ].addEvent(doSwInit,            swInitializing,    kPublicEvent); // Alternative: safetySys.level(off).addEvent(doSwInit, swInitializing, kPublicEvent);
	safetySys[swShutingDown    ].addEvent(swShutDownDone,      off,               kPrivateEvent);
	safetySys[swInitializing   ].addEvent(swInitDone,          swInitialized,     kPrivateEvent);
	safetySys[swInitialized    ].addEvent(doOff,               swShutingDown,     kPublicEvent);
	safetySys[swInitialized    ].addEvent(doControlStart,      controlStarting,   kPublicEvent);
	safetySys[controlStopping  ].addEvent(controlStoppingDone, swInitialized,     kPrivateEvent);
	safetySys[controlStarting  ].addEvent(controlStartingDone, systemOn,          kPrivateEvent);
	safetySys[emergency        ].addEvent(doEmergencyReset,    resetingEmergency, kPublicEvent);
	safetySys[resetingEmergency].addEvent(emergencyResetDone,  systemOn,          kPrivateEvent);
	safetySys[systemOn         ].addEvent(doStopControl,       controlStopping,   kPublicEvent);
	safetySys[systemOn         ].addEvent(doPoweringUp,        poweringUp,        kPublicEvent);
	safetySys[poweringDown     ].addEvent(poweringDownDone,    systemOn,          kPrivateEvent);
	safetySys[poweringUp       ].addEvent(poweringUpDone,      powerOn,           kPrivateEvent);
	safetySys[powerOn          ].addEvent(doPoweringDown,      poweringDown,      kPublicEvent);
	safetySys[powerOn          ].addEvent(doStartingMotion,    motionStarting,    kPublicEvent);
	safetySys[motionStopping   ].addEvent(motionStoppingDone,  powerOn,           kPrivateEvent);
	safetySys[motionStarting   ].addEvent(motionStartingDone,  moving,            kPrivateEvent);
	safetySys[moving           ].addEvent(doMotionStopping,    motionStopping,    kPublicEvent);
	
	// Add events to multiple levels
	safetySys.addEventToLevelAndAbove(powerOn, doEmergency, emergency, kPublicEvent);
	
	// Define input states and events for all levels
	safetySys[off              ].setInputActions({ ignore(emergencyStop),                   ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[swShutingDown    ].setInputActions({ ignore(emergencyStop),                   ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[swInitializing   ].setInputActions({ ignore(emergencyStop),                   ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[swInitialized    ].setInputActions({ ignore(emergencyStop),                   ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[controlStopping  ].setInputActions({ ignore(emergencyStop),                   ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[controlStarting  ].setInputActions({ ignore(emergencyStop),                   ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[emergency        ].setInputActions({ ignore(emergencyStop),                   ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[resetingEmergency].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[systemOn         ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[poweringDown     ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[poweringUp       ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[powerOn          ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[motionStopping   ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[motionStarting   ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q[0]),                              ignore(q[1]),                              ignore(q[2]),                              ignore(q[3])                              });
	safetySys[moving           ].setInputActions({ check(emergencyStop, true, doEmergency), range(q[0], 0.0, 6.283, doMotionStopping), range(q[1], 0.0, 6.283, doMotionStopping), range(q[2], 0.0, 6.283, doMotionStopping), range(q[3], 0.0, 6.283, doMotionStopping) });
	
	// Define output states and events for all levels
	safetySys[off              ].setOutputActions({ set(power, false), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[swShutingDown    ].setOutputActions({ set(power, false), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[swInitializing   ].setOutputActions({ set(power, false), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[swInitialized    ].setOutputActions({ set(power, false), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[controlStopping  ].setOutputActions({ set(power, false), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[controlStarting  ].setOutputActions({ set(power, false), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[emergency        ].setOutputActions({ set(power, false), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[resetingEmergency].setOutputActions({ set(power, false), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[systemOn         ].setOutputActions({ set(power, true ), set(enable[0], false), set(enable[1], false), set(enable[2], false), set(enable[3], false), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[poweringDown     ].setOutputActions({ set(power, true ), set(enable[0], true ), set(enable[1], true ), set(enable[2], true ), set(enable[3], true ), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[poweringUp       ].setOutputActions({ set(power, true ), set(enable[0], true ), set(enable[1], true ), set(enable[2], true ), set(enable[3], true ), set(brake[0], true ), set(brake[1], true ), set(brake[2], true ), set(brake[3], true ) });
	safetySys[powerOn          ].setOutputActions({ set(power, true ), set(enable[0], true ), set(enable[1], true ), set(enable[2], true ), set(enable[3], true ), set(brake[0], false), set(brake[1], false), set(brake[2], false), set(brake[3], false) });
	safetySys[motionStopping   ].setOutputActions({ set(power, true ), set(enable[0], true ), set(enable[1], true ), set(enable[2], true ), set(enable[3], true ), set(brake[0], false), set(brake[1], false), set(brake[2], false), set(brake[3], false) });
	safetySys[motionStarting   ].setOutputActions({ set(power, true ), set(enable[0], true ), set(enable[1], true ), set(enable[2], true ), set(enable[3], true ), set(brake[0], false), set(brake[1], false), set(brake[2], false), set(brake[3], false) });
	safetySys[moving           ].setOutputActions({ set(power, true ), set(enable[0], true ), set(enable[1], true ), set(enable[2], true ), set(enable[3], true ), set(brake[0], false), set(brake[1], false), set(brake[2], false), set(brake[3], false) });
	
			
	// Define and add level functions
	safetySys[swInitializing].setLevelAction([&](SafetyContext* privateContext) {
		safetySys.triggerEvent(swInitDone, privateContext);
	});
	
	// Define entry level
	safetySys.setEntryLevel(off);
	
	std::cout << "Example done..." << std::endl;
}
