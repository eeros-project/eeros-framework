#pragma once

#include <eeros/core/Runnable.hpp>
#include <eeros/sequencer/Transitions.hpp>

//Forward Declarations
class Sequence;

class SequencerStep : public Runnable
{
protected:
	Transitions* trans;
	string nameOfStep;

	//Sequence zu der dieser Step gehört
	Sequence* ownerSequence;
public:
	SequencerStep(Transitions* p_trans, string name, Sequence* owner);
	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~SequencerStep(void);
	Transitions* getTransitons();
	list<string>& getAllowedTransitions();
	string getName();
	void fillAllowedStepName(string name);
	virtual void run();
};

