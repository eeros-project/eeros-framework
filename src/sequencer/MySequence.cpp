#include "StdAfx.h"
#include "MySequence.hpp"

#include "Initialising.hpp"
#include "Initialised.hpp"
#include "Homed.hpp"
#include "Stopping.hpp"

MySequence::MySequence(double period, string name) : 
Sequence(period, name)
{
	fillSequencerSteps();
}


MySequence::~MySequence(void)
{
}

void MySequence::fillSequencerSteps(){
	//Init wird immer hinzugefügt und wechselt zu Intialising
	//Step Initialising darf nur nach Initialised übergehen.
	Transitions* trans = new Transitions();
	trans->addSequencerStepName("Initialised");
	SequencerStep* step = new Initialising(trans, "Initialising", this);
	addSequencerStep(step);

	trans = new Transitions();
	trans->addSequencerStepName("Homed");
	step = new Initialised(trans, "Initialised", this);
	addSequencerStep(step);

	trans = new Transitions();
	trans->addSequencerStepName("Stopping");
	step = new Homed(trans, "Homed", this);
	addSequencerStep(step);

	trans = new Transitions();
	trans->addSequencerStepName("");
	step = new Stopping(trans, "Stopping", this);
	addSequencerStep(step);

}