
#include <eeros/core/Runnable.hpp>

using namespace std;
#include <string>

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/SequencerStep.hpp>



SequencerStep::SequencerStep(Transitions* p_trans, string name, Sequence* owner) :
	callingSubSequence(0),
	ownerSequence(owner),
	trans(p_trans),
	nameOfStep(name),
	waitForSequenceStop(false)
{
}


SequencerStep::~SequencerStep(void)
{
	delete trans;
}

Transitions* SequencerStep::getTransitons(){
	return trans;
}

string SequencerStep::getName(){
	return nameOfStep;
}

void SequencerStep::startSubSequence(Sequence* p_subSequence, bool waitForTermination){
	p_subSequence->start();
	waitForSequenceStop = waitForTermination;
	callingSubSequence = p_subSequence;
}

void SequencerStep::run(){
	//Als Beispiel für run warten auf SubSequence
	if(callingSubSequence && waitForSequenceStop){
		if(callingSubSequence->getStatus() == Executor::kRunning && waitForSequenceStop){
			return;
		}else if(callingSubSequence->getStatus() == Executor::kStopped){
			callingSubSequence = 0;
			waitForSequenceStop = false;
		}
	}else if(callingSubSequence){
		ownerSequence->addSubSequence(callingSubSequence);
	}
	//go on run
}

void SequencerStep::waitForSequenceEnd(string name){
	Sequence* waitSequence = ownerSequence->findSequence(name);
	if(waitSequence){
		WaitForSingleObject( ExecutorService::getHandle(ownerSequence->getThreadId()), INFINITE);
	}
}