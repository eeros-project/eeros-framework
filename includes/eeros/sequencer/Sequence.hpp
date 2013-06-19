#pragma once

#include <eeros/core/Executor.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/sequencer/Transitions.hpp>
#include <eeros/sequencer/SequencerStep.hpp>


using namespace std;

/** Sequence wird als Thread aufgerufen, aber nciht periodisch
*/

class Sequence : public Executor
{
private:
	string sequenceName;
	/** Controlstruktur ist in der Liste von Executor list<Runnable*>
	*/
	//list<Block*> blockes;

	/** currentStep ist als einziger Eintrag in runnables vom Executor und wird umgebogen
	*/
	SequencerStep* currentStep;
	

	/** alle Schritte dieser Sequence sind in dieser Liste
	*/
	list<SequencerStep*> sequencerSteps;

	list<Sequence*> subSequences;
	void deleteSubSequences();

	//void deleteBlockList();
	void deleteSequencerStepList();
	//in C++ geht das überschreiben einer privaten MEthode!
	virtual void run();

public:
	Sequence(double period, string name);

	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~Sequence(void);
	//list<Block*>& getBlockList();
	void safeTransition(string nameOfSequenceStep);
	void addSubSequence(Sequence* subSequence);
	void addSequencerStep(SequencerStep* sequencerStep);
	Sequence* findSequence(string name);
	string getName();
	virtual void fillSequencerSteps() = 0;
};

