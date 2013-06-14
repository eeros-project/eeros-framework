//TODO
#include <iostream>

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Homed.hpp>


Homed::Homed(Transitions* p_trans, string name, Sequence* owner) :
SequencerStep(p_trans, name, owner)
{
}


Homed::~Homed(void)
{
}

void Homed::run(){
	//TODO
	ownerSequence->safeTransition("Stopping");
	//für TEst
	cout << "Going to Initialising" << endl;
}