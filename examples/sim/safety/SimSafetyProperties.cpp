#include "SimSafetyProperties.hpp"
#include "../control/SimControlSystem.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/safety/ControlInput.hpp>
#include <eeros/math/Matrix.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>
#include <cmath>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

SimSafetyProperties::SimSafetyProperties(SimControlSystem* cs) : controlSys(cs), 
	  slOff("System off"),
	  slRunning("running"),
	  
	  seRun("start running")
	 {

// 	  HAL& hal = HAL::instance();
  
	// ############ Define critical outputs ############
	
	// ############ Define critical inputs ############
	
	// ############ Define Levels ############
	
	addLevel(slOff);
	addLevel(slRunning);
	
	// ############ Add events to the levels ############
	
	// ############ Define input states and events for all levels ############
	slOff.addEvent(seRun, slRunning, kPublicEvent);
	
	// Define output states and events for all levels 

	// *** Define and add level functions *** //
	slOff.setLevelAction([&](SafetyContext* privateContext){
		static int cnt = 0;
		cnt++;
		if(cnt > 2000){
			privateContext->triggerEvent(seRun);
		}
	});
	
	// Define entry level
	setEntryLevel(slOff);
}

SimSafetyProperties::~SimSafetyProperties() {
	// nothing to do
}