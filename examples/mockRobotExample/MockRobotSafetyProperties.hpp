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
		cs(cs),
		slOff("off"),
		slHoming("homing"),
		slReady("ready"),
		slMoving("moving"),
		abort("abort"),
		homingDone("homing done"),
		startMoving("start moving")
	{
		addLevel(slOff);
		addLevel(slHoming);
		addLevel(slReady);
		addLevel(slMoving);
		
		slHoming.addEvent(homingDone, slReady, kPublicEvent);
		slReady.addEvent(startMoving, slMoving, kPublicEvent);
		addEventToLevelAndAbove(slHoming, abort, slOff, kPublicEvent);
		
		slOff.setLevelAction([&](SafetyContext* privateContext) {eeros::Executor::stop();});
		
		slHoming.setLevelAction([&](SafetyContext* privateContext) {
			if (slHoming.getNofActivations() == 1)
				Sequencer::instance().getSequenceByName("Homing Sequence")->start();
			if (cs.iX.getOut().getSignal().getValue() >= 1.0) cs.setpointX.setValue(0);
			if (cs.iY.getOut().getSignal().getValue() >= 1.0) cs.setpointY.setValue(0);
			if (Sequencer::instance().getSequenceByName("Homing Sequence")->getRunningState() == SequenceState::terminated)
				privateContext->triggerEvent(homingDone);
		});
		
		slReady.setLevelAction([=](SafetyContext* privateContext) {
			if (slReady.getNofActivations() * ts >= 2)
				privateContext->triggerEvent(startMoving);
		});
		
		slMoving.setLevelAction([&](SafetyContext* privateContext) {
			if (slMoving.getNofActivations() == 1)
				Sequencer::instance().getSequenceByName("UpAndDown Sequence")->start();
		});
		
		setEntryLevel(slHoming);
		
		exitFunction = ([&](SafetyContext* privateContext){
			privateContext->triggerEvent(abort);
		});

	}
	virtual ~MockRobotSafetyProperties() { }
	
	SafetyEvent abort;
	SafetyEvent homingDone;
	SafetyEvent startMoving;

	SafetyLevel slOff;
	SafetyLevel slHoming;
	SafetyLevel slReady;
	SafetyLevel slMoving;
	
	MockRobotControlSystem& cs;
};

#endif // MOCK_ROBOT_SAFETY_PROPERTIES_HPP_
