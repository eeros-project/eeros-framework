#include "MySequence.hpp"

#include <eeros/sequencer/Initialising.hpp>
#include <eeros/sequencer/Initialised.hpp>
#include <eeros/sequencer/Homed.hpp>
#include <eeros/sequencer/Stopping.hpp>

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
	trans->addAllowedTransitionName("Initialised");
	SequencerStep* step = new Initialising(trans, "Initialising", this);
	addSequencerStep(step);

	trans = new Transitions();
	trans->addAllowedTransitionName("Homed");
	step = new Initialised(trans, "Initialised", this);
	addSequencerStep(step);

	trans = new Transitions();
	trans->addAllowedTransitionName("Stopping");
	step = new Homed(trans, "Homed", this);
	addSequencerStep(step);

	trans = new Transitions();
	trans->addAllowedTransitionName("");
	step = new Stopping(trans, "Stopping", this);
	addSequencerStep(step);

}