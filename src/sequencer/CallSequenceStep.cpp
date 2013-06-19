
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/CallSequenceStep.hpp>

//TODELETE
#include <iostream>


CallSequenceStep::CallSequenceStep(Transitions* p_trans, string name, Sequence* owner, bool waiting, Sequence* calling) :
	SequencerStep(p_trans, name, owner),
	waitForSequenceStop(waiting)
{
	this->callingSubSequence = calling;
}


CallSequenceStep::~CallSequenceStep(void)
{
}

void CallSequenceStep::run(){
	//Als Beispiel für run warten auf SubSequence
	if(callingSubSequence && callingSubSequence->getStatus() == kStopped){
		callingSubSequence->start();
	}
	if(callingSubSequence && waitForSequenceStop){
		if(callingSubSequence->getStatus() == kRunning && waitForSequenceStop){
			return;
		}else if(callingSubSequence->getStatus() == kStopped){
			callingSubSequence = 0;
			waitForSequenceStop = false;
		}
	}else if(callingSubSequence){
		ownerSequence->addSubSequence(callingSubSequence);
		callingSubSequence = 0;
	}
	//TODO
	//nächster Step
	SequencerStep::run();
}
