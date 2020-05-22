#ifndef MOCK_ROBOT_SAFETY_PROPERTIES_HPP_
#define MOCK_ROBOT_SAFETY_PROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include "MockRobotControlSystem.hpp"

using namespace eeros::safety;
using namespace eeros::sequencer;

class MockRobotSafetyProperties : public SafetyProperties {
public:
	MockRobotSafetyProperties(MockRobotControlSystem& cs, double ts) : 
		slOff("off"),
		slHoming("homing"),
		slReady("ready"),
		slMoving("moving"),
		abort("abort"),
		homingDone("homing done"),
		startMoving("start moving"),
		cs(cs)
	{
		addLevel(slOff);
		addLevel(slHoming);
		addLevel(slReady);
		addLevel(slMoving);
		
		slHoming.addEvent(homingDone, slReady, kPublicEvent);
		slReady.addEvent(startMoving, slMoving, kPublicEvent);
		addEventToLevelAndAbove(slHoming, abort, slOff, kPrivateEvent);
		
		slOff.setLevelAction([&](SafetyContext* privateContext) {eeros::Executor::stop();});
		
		setEntryLevel(slHoming);
		
		exitFunction = ([&](SafetyContext* privateContext){
			privateContext->triggerEvent(abort);
		});

	}
	virtual ~MockRobotSafetyProperties() { }
	
	SafetyLevel slOff;
	SafetyLevel slHoming;
	SafetyLevel slReady;
	SafetyLevel slMoving;
	
	SafetyEvent abort;
	SafetyEvent homingDone;
	SafetyEvent startMoving;

	MockRobotControlSystem& cs;
};

#endif // MOCK_ROBOT_SAFETY_PROPERTIES_HPP_
