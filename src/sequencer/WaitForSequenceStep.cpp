
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/WaitForSequenceStep.hpp>


WaitForSequenceStep::WaitForSequenceStep(Transitions* p_trans, string name, Sequence* owner, Sequence* subSequence) :
	SequencerStep(p_trans, name, owner), 
		waitSequence(subSequence)
{
}


WaitForSequenceStep::~WaitForSequenceStep(void)
{
}

void WaitForSequenceStep::run(){
	ExecutorService::waitForSequenceEnd(waitSequence);
	//go on run
	SequencerStep::run();
}


