#pragma once

#include "eeros/core/Runnable.hpp"
//TODO Pfad anpassen
#include "Transitions.hpp"
//#include "Sequence.hpp"

//Forward Declarations
class Sequence;

class SequencerStep : public Runnable
{
protected:
	Transitions* trans;
	string nameOfStep;

	//SubSequence welche von diesem Step gestartet wird
	Sequence* callingSubSequence;
	bool waitForSequenceStop;

	//Sequence zu der dieser Step gehört
	Sequence* ownerSequence;
public:
	SequencerStep(Transitions* p_trans, string name, Sequence* owner);
	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~SequencerStep(void);
	Transitions* getTransitons();
	string getName();
	void fillAllowedStepName(string name);
	/** startet eine Subsequenc und wartet falls waitForTermination = true im Step bis sie beendet wird
	 *  sonst wird die nächste Transition ausgeführt
	 */
	void startSubSequence(Sequence* p_subSequence, bool waitForTermination);
	virtual void run();
	void waitForSequenceEnd(string name);
};

