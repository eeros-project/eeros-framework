#pragma once


#include <eeros/sequencer/SequencerStep.hpp>

class CallSequenceStep : public SequencerStep
{
private:
	//SubSequence welche von diesem Step gestartet wird
	Sequence* callingSubSequence;
	// Soll in diesem STep gewartet werden?
	bool waitForSequenceStop;
public:
	CallSequenceStep(Transitions* p_trans, std::string name, Sequence* owner, bool waiting, Sequence* calling);
	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~CallSequenceStep(void);
	virtual void run();
	/** startet eine Subsequence und wartet falls waitForTermination = true im Step bis sie beendet wird
	 *  sonst wird die nächste Transition ausgeführt
	 */
	void startSubSequence();
};

