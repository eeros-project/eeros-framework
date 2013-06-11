#include "StdAfx.h"

//TODO Pfad anpassen
#include "Sequence.hpp"
#include "Init.hpp"


Sequence::Sequence(double period, string name) :
	Executor(period),
	sequenceName(name)
{
	Transitions* trans = new Transitions();
	trans->addSequencerStepName("Initialising");
	Init* init = new Init(trans, "Init", this);
	sequencerSteps.push_back(init);
	currentStep = init;
}

Sequence::~Sequence(void)
{
	//deleteBlockList();
	deleteSequencerStepList();
	deleteSubSequences();
}


/*void Sequence::deleteBlockList(){
	list<Block*>::reverse_iterator rev_iter = blockes.rbegin();
	while(rev_iter != blockes.rend()){
		delete (*rev_iter);
		rev_iter++;
	}
	blockes.clear();
}
*/

void Sequence::deleteSequencerStepList(){
	list<SequencerStep*>::reverse_iterator rev_iter = sequencerSteps.rbegin();
	while(rev_iter != sequencerSteps.rend()){
		delete (*rev_iter);
		rev_iter++;
	}
	sequencerSteps.clear();
}

/*list<Block*>& Sequence::getBlockList(){
	return blockes;
}
*/

void Sequence::safeTransition(string nameOfDesiredSequenceStep){
	if(nameOfDesiredSequenceStep.compare("") == 0){
		currentStep = 0;
		return;
	}
	if( currentStep->getTransitons()->isTransitionAllowed(nameOfDesiredSequenceStep) ){
		list<SequencerStep*>::iterator iter = sequencerSteps.begin();
		while(iter != sequencerSteps.end()){
			if((*iter)->getName().compare(nameOfDesiredSequenceStep) == 0){
				currentStep = *iter;
				return;
			}
			iter++;
		}
	}
}

/** run wird in einer while Schleife vom ExecutorService aufgerufen
 *  private Methoden überschreiben geht in C++
 */
void Sequence::run(){
	if(currentStep){
		currentStep->run();
	}
}

void Sequence::deleteSubSequences(){
	list<Sequence*>::reverse_iterator rev_iter = subSequences.rbegin();
	while(rev_iter != subSequences.rend()){
		delete (*rev_iter);
		rev_iter++;
	}
	subSequences.clear();
}

void Sequence::addSubSequence(Sequence* subSequence){
	subSequences.push_back(subSequence);
}

Sequence* Sequence::findSequence(string name){
	list<Sequence*>::iterator iter = subSequences.begin();
		while(iter != subSequences.end()){
			if((*iter)->getName().compare(name) == 0){
				return *iter;
			}
			iter++;
		}
		return 0;
}

string Sequence::getName(){
	return sequenceName;
}

void Sequence::addSequencerStep(SequencerStep* sequencerStep){
	sequencerSteps.push_back(sequencerStep);
}