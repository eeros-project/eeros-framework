#include <iostream>
#include <vector>
#include <initializer_list>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/SafetyLevel.hpp>

int main() {
	std::cout << "Safety System Example started..." << std::endl;
	
	// 1) Get Safety System instance
	SafetySystem& safetySys = SafetySystem::instance();
	
	// 2) Define all possible events
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
	
	// 3) Name all levels
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

	// 4) Create a list of all levels
	std::vector<SafetyLevel> levels {
		{ off,               "Ausgeschalteter Zustand" },
		{ swShutingDown,     "Software herunterfahren" },
		{ swInitializing,    "Aufstarten der Software" },
		{ swInitialized,     "Software ist initialisiert" },
		{ controlStopping,   "Reglertask wird gestoppt" },
		{ controlStarting,   "Reglertask wird gestartet" },
		{ emergency,         "Notauszustand" },
		{ resetingEmergency, "Zurücksetzen des Notaus" },
		{ systemOn,          "System ist bereit, aber noch ohne Leistung", { {"wd", true}, {"power", true} } },
		{ poweringDown,      "Leistung ausschalten",                       { {"wd", true}, {"power", true}, {"enable", true} } },
		{ poweringUp,        "Leistung einschalten",                       { {"wd", true}, {"power", true}, {"enable", true} } },
		{ powerOn,           "Leistung ist ein, Motoren werden geregelt",  { {"wd", true}, {"power", true}, {"enable", true}, {"break", true} } },
		{ motionStopping,    "",                                           { {"wd", true}, {"power", true}, {"enable", true}, {"break", true} } },
		{ motionStarting,    "",                                           { {"wd", true}, {"power", true}, {"enable", true}, {"break", true} } },
		{ moving,            "",                                           { {"wd", true}, {"power", true}, {"enable", true}, {"break", true} } },
	};
	
	// 5) Set the levels in the safety system
	safetySys.setSafetyLevels(levels);
	
	// 6) Add events to each level
	safetySys[off].addEvent(doSwInit, swInitializing);  // Alternative: safetySys.level(off).addEvent(doSwInit, swInitializing);
	
	safetySys[swShutingDown].addEvent(swShutDownDone, off);
	safetySys[swShutingDown].setExitEvent(off);
	
	safetySys[swInitializing].addEvent(swInitDone, swInitialized);
	safetySys[swInitializing].setExitEvent(swInitDone);
	
	safetySys[swInitialized].addEvent(doOff, swShutingDown);
	safetySys[swInitialized].addEvent(doControlStart, controlStarting);
	
	safetySys[controlStopping].addEvent(controlStoppingDone, swInitialized);
	safetySys[controlStopping].setExitEvent(controlStoppingDone);
	
	safetySys[controlStarting].addEvent(controlStartingDone, systemOn);
	safetySys[controlStarting].setExitEvent(controlStartingDone);
	
	safetySys[emergency].addEvent(doEmergencyReset, resetingEmergency);
	
	safetySys[resetingEmergency].addEvent(emergencyResetDone, systemOn);
	safetySys[resetingEmergency].setExitEvent(emergencyResetDone);
	
	safetySys[systemOn].addEvent(doStopControl, controlStopping);
	safetySys[systemOn].addEvent(doPoweringUp, poweringUp);
	
	safetySys[poweringDown].addEvent(poweringDownDone, systemOn);
	safetySys[poweringDown].setExitEvent(poweringDownDone);
	
	safetySys[poweringUp].addEvent(poweringUpDone, powerOn);
	safetySys[poweringUp].setExitEvent(poweringUpDone);
	
	safetySys[powerOn].addEvent(doPoweringDown, poweringDown);
	safetySys[powerOn].addEvent(doStartingMotion, motionStarting);
	
	safetySys[motionStopping].addEvent(motionStoppingDone, powerOn);
	safetySys[motionStopping].setExitEvent(motionStoppingDone);
	
	safetySys[motionStarting].addEvent(motionStartingDone, moving);
	safetySys[motionStarting].setExitEvent(motionStartingDone);
	
	safetySys[moving].addEvent(doMotionStopping, motionStopping);
	
	// 7) Add events to multiple levels
	safetySys.addEventToAllLevelsAbove(powerOn, doEmergency, emergency);
	
	// 8) Define and add level functions
	safetySys[swInitializing].setLevelAction([]() { 
		int a = 0;
		int b = 5;
	});
	
	std::cout << "Example done..." << std::endl;
}
