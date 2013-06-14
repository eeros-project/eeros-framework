#pragma once

#include <eeros\sequencer\SequencerStep.hpp>
#include <eeros\sequencer\Initialising.hpp>

class Initialising : public SequencerStep
{
public:
	Initialising(Transitions* p_trans, string name, Sequence* owner);
	virtual ~Initialising(void);
	void fillBlocks();
	void connectBlocks();
	virtual void run();
};

