#include <eeros/sequencer/Initialising.hpp>
#include <eeros/sequencer/Initialised.hpp>
#include <eeros/sequencer/Homed.hpp>
#include <eeros/sequencer/Stopping.hpp>
#include <eeros/sequencer/CallSequenceStep.hpp>
#include <eeros/sequencer/WaitForSequenceStep.hpp>

#include "MySequence.hpp"
#include "MySubSequence.hpp"

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

	trans = new Transitions();
	trans->addAllowedTransitionName("CallSequenceStep");
	step = new Initialised(trans, "Initialised", this);

	trans = new Transitions();
	trans->addAllowedTransitionName("Homed");
	Sequence* subSequence = new MySubSequence(1.0, "SubSequence");
	//So wird im Step automatisch gewartet bis der Sequence Thread beendet wird
	//step = new CallSequenceStep(trans, "CallSequenceStep", this, true, subSequence);

	//So wird nicht im Step  gewartet bis der Sequence Thread beendet wird
	//auf das Ende warten im Step WaitforsequenceStep
	step = new CallSequenceStep(trans, "CallSequenceStep", this, false, subSequence);

	trans = new Transitions();
	trans->addAllowedTransitionName("WaitForSequence");
	step = new Homed(trans, "Homed", this);

	trans = new Transitions();
	trans->addAllowedTransitionName("Stopping");
	step = new WaitForSequenceStep(trans, "WaitForSequence", this, subSequence);

	trans = new Transitions();
	trans->addAllowedTransitionName("");
	step = new Stopping(trans, "Stopping", this);

}