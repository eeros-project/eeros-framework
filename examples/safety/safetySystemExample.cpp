#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/core/EEROSException.hpp>
#include <eeros/hal/ComediDigIn.hpp>
#include <eeros/hal/ComediDigOut.hpp>
#include <eeros/hal/ComediFqd.hpp>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

void initHardware() {
	HAL& hal = HAL::instance();
	
	// Define system in- and outputs
	ComediDevice* comedi0 = new ComediDevice("/dev/comedi0");
	
	// Add system in- and outputs to the HAL
	hal.addSystemInput(new ComediFqd("q0", comedi0, 11, 8, 10, 9, 6.28318530718 / 2000.0, 0, 0));
	hal.addSystemInput(new ComediFqd("q1", comedi0, 11, 3, 11, 4, 6.28318530718 / 2000.0, 0, 0));
	hal.addSystemInput(new ComediDigIn("emergencyStop", comedi0, 2, 0));
	hal.addSystemOutput(new ComediDigOut("enable0", comedi0, 2, 8));
	hal.addSystemOutput(new ComediDigOut("enable1", comedi0, 2, 9));
	hal.addSystemOutput(new ComediDigOut("brake0", comedi0, 2, 10));
	hal.addSystemOutput(new ComediDigOut("brake1", comedi0, 2, 11));
	hal.addSystemOutput(new ComediDigOut("power", comedi0, 2, 12));
}

int main() {
	std::cout << "Safety System Example started..." << std::endl;
	
	// Get Safety System instance
	SafetySystem& safetySys = SafetySystem::instance();
	
	// Get HAL instance
	HAL& hal = HAL::instance();
	
	// Initialize Hardware
	initHardware();
	
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
	SystemOutput<bool>* power = hal.getLogicSystemOutput("power");
	
	SystemOutput<bool>* enable0 = hal.getLogicSystemOutput("enable0");
	SystemOutput<bool>* enable1 = hal.getLogicSystemOutput("enable1");

	SystemOutput<bool>* brake0 = hal.getLogicSystemOutput("brake0");
	SystemOutput<bool>* brake1 = hal.getLogicSystemOutput("brake1");

	safetySys.defineCriticalOutputs({
		power,
		enable0,
		enable1,
		brake0,
		brake1,
	});
	
	// Define criticcal inputs
	SystemInput<bool>* emergencyStop = hal.getLogicSystemInput("emergencyStop");
	SystemInput<double>* q0 = hal.getRealSystemInput("q0");
	SystemInput<double>* q1 = hal.getRealSystemInput("q1");
	
	safetySys.defineCriticalInputs({
		emergencyStop,
		q0,
		q1
	});
		
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
	safetySys[off              ].setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	safetySys[swShutingDown    ].setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	safetySys[swInitializing   ].setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	safetySys[swInitialized    ].setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	safetySys[controlStopping  ].setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	safetySys[controlStarting  ].setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	safetySys[emergency        ].setInputActions({ ignore(emergencyStop),                   ignore(q0),                              ignore(q1)                              });
	safetySys[resetingEmergency].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	safetySys[systemOn         ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	safetySys[poweringDown     ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	safetySys[poweringUp       ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	safetySys[powerOn          ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	safetySys[motionStopping   ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	safetySys[motionStarting   ].setInputActions({ check(emergencyStop, true, doEmergency), ignore(q0),                              ignore(q1)                              });
	safetySys[moving           ].setInputActions({ check(emergencyStop, true, doEmergency), range(q0, 0.0, 6.283, doMotionStopping), range(q1, 0.0, 6.283, doMotionStopping) });
	
	// Define output states and events for all levels
	safetySys[off              ].setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[swShutingDown    ].setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[swInitializing   ].setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[swInitialized    ].setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[controlStopping  ].setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[controlStarting  ].setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[emergency        ].setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[resetingEmergency].setOutputActions({ set(power, false), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[systemOn         ].setOutputActions({ set(power, true ), set(enable0, false), set(enable1, false), set(brake0, true ), set(brake1, true ) });
	safetySys[poweringDown     ].setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, true ), set(brake1, true ) });
	safetySys[poweringUp       ].setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, true ), set(brake1, true ) });
	safetySys[powerOn          ].setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, false), set(brake1, false) });
	safetySys[motionStopping   ].setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, false), set(brake1, false) });
	safetySys[motionStarting   ].setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, false), set(brake1, false) });
	safetySys[moving           ].setOutputActions({ set(power, true ), set(enable0, true ), set(enable1, true ), set(brake0, false), set(brake1, false) });
	
			
	// Define and add level functions
	safetySys[swInitializing].setLevelAction([&](SafetyContext* privateContext) {
		//safetySys.triggerEvent(swInitDone, privateContext);
	});
	
	// Define entry level
	safetySys.setEntryLevel(off);
	
	std::cout << "Starting executor..." << std::endl;
	Executor e(1);
	e.addRunnable(safetySys);
	e.start();
	
	sleep(20);
	
	std::cout << "Stopping executor..." << std::endl;
	e.stop();
	while(!e.isTerminated());
	
	std::cout << "Example done..." << std::endl;
}
