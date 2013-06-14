#pragma once


#include <eeros/sequencer/SequencerStep.hpp>

class Init : public SequencerStep
{
public:
	Init(Transitions* p_trans, string name, Sequence* owner);
	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~Init(void);
	virtual void run();
};

