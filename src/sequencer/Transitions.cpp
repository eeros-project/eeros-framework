
#include <eeros/sequencer/Transitions.hpp>
#include <eeros/sequencer/SequencerStep.hpp>

Transitions::Transitions() 
{
}


Transitions::~Transitions(void)
{
	deleteSequencerStepsName();
}

void Transitions::addAllowedTransitionName(string name){
	allowedSteps.push_back(name);
}

void Transitions::deleteSequencerStepsName(){
	allowedSteps.clear();
}

bool Transitions::isTransitionAllowed(string name){
	list<string>::iterator iter = allowedSteps.begin();
	while(iter != allowedSteps.end()){
		if((*iter).compare(name) == 0){
			return true;
		}
		iter++;
	}
	return false;
}
