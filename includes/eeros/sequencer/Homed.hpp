#pragma once

#include <eeros/sequencer/SequencerStep.hpp>

class Homed : public SequencerStep
{
public:
	Homed(Transitions* p_trans, string name, Sequence* owner);
	virtual ~Homed(void);
	virtual void run();
};

