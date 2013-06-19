#pragma once

#include <eeros/sequencer/SequencerStep.hpp>

class Initialised : public SequencerStep
{
public:
	Initialised(Transitions* p_trans, std::string name, Sequence* owner);
	virtual ~Initialised(void);
	virtual void run();
};

