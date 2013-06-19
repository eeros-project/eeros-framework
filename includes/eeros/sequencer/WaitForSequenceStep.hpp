#pragma once


#include <eeros/sequencer/SequencerStep.hpp>

class WaitForSequenceStep : public SequencerStep
{
private:
	Sequence* waitSequence;
public:
	WaitForSequenceStep(Transitions* p_trans, string name, Sequence* owner, Sequence* subSequence);
	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~WaitForSequenceStep(void);
	virtual void run();
};

