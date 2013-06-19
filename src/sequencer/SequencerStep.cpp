
#include <eeros/core/Runnable.hpp>

using namespace std;
#include <string>

//TODELETE
#include <iostream>

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/SequencerStep.hpp>



SequencerStep::SequencerStep(Transitions* p_trans, string name, Sequence* owner) :
	ownerSequence(owner),
	trans(p_trans),
	nameOfStep(name)
{
	ownerSequence->addSequencerStep(this);
}


SequencerStep::~SequencerStep(void)
{
	delete trans;
}

Transitions* SequencerStep::getTransitons(){
	return trans;
}

list<string>& SequencerStep::getAllowedTransitions(){
	return trans->allowedSteps;
}

string SequencerStep::getName(){
	return nameOfStep;
}

void SequencerStep::run(){
	//TODO
	string nextTrans;
	list<string>& allowedTrans = getAllowedTransitions();
	list<string>::iterator iter = allowedTrans.begin();
	nextTrans = *iter;
	ownerSequence->safeTransition(nextTrans);
	cout << "Going to " << nextTrans << endl;
}

