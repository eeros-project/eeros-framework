#include "MySubSequence.hpp"

#include <eeros/sequencer/Initialising.hpp>
#include <eeros/sequencer/Initialised.hpp>
#include <eeros/sequencer/Homed.hpp>
#include <eeros/sequencer/Stopping.hpp>


MySubSequence::MySubSequence(double period, string name) : 
Sequence(period, name)
{
	fillSequencerSteps();
}


MySubSequence::~MySubSequence(void)
{
}

void MySubSequence::fillSequencerSteps(){
	//Init wird immer hinzugefügt und wechselt zu Intialising
	//Step Initialising darf nur nach Initialised übergehen.
	Transitions* trans = new Transitions();
	trans->addAllowedTransitionName("Initialised");
	SequencerStep* step = new Initialising(trans, "Initialising", this);

	trans = new Transitions();
	trans->addAllowedTransitionName("Homed");
	step = new Initialised(trans, "Initialised", this);

	trans = new Transitions();
	trans->addAllowedTransitionName("Stopping");
	step = new Homed(trans, "Homed", this);

	trans = new Transitions();
	trans->addAllowedTransitionName("");
	step = new Stopping(trans, "Stopping", this);
}