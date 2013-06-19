#pragma once

#include <string>
#include <list>

#include <eeros/core/Runnable.hpp>
#include <eeros/sequencer/Transitions.hpp>

//Forward Declarations
class Sequence;

class SequencerStep : public Runnable
{
protected:
	Transitions* trans;
	std::string nameOfStep;

	//Sequence zu der dieser Step gehört
	Sequence* ownerSequence;
public:
	SequencerStep(Transitions* p_trans, std::string name, Sequence* owner);
	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~SequencerStep(void);
	Transitions* getTransitons();
	std::list<std::string>& getAllowedTransitions();
	std::string getName();
	void fillAllowedStepName(std::string name);
	virtual void run();
};

