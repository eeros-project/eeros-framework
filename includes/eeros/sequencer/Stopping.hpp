#pragma once

#include <eeros/sequencer/SequencerStep.hpp>

class Stopping: public SequencerStep
{
public:
	Stopping(Transitions* p_trans, std::string name, Sequence* owner);
	virtual ~Stopping(void);
	virtual void run();
};

