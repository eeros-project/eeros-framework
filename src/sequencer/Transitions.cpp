
#include <eeros/sequencer/Transitions.hpp>
#include <eeros/sequencer/SequencerStep.hpp>

Transitions::Transitions() 
{
}


Transitions::~Transitions(void)
{
	deleteSequencerStepsName();
}

void Transitions::addAllowedTransitionName(std::string name){
	allowedSteps.push_back(name);
}

void Transitions::deleteSequencerStepsName(){
	allowedSteps.clear();
}

bool Transitions::isTransitionAllowed(std::string name){
	std::list<std::string>::iterator iter = allowedSteps.begin();
	while(iter != allowedSteps.end()){
		if((*iter).compare(name) == 0){
			return true;
		}
		iter++;
	}
	return false;
}
