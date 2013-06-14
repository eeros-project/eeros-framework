
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Initialising.hpp>

//TODELETE
#include <iostream>

Initialising::Initialising(Transitions* p_trans, string name, Sequence* owner) : 
SequencerStep(p_trans, name, owner)
{
}


Initialising::~Initialising(void)
{
}

void Initialising::fillBlocks(){
}

void Initialising::connectBlocks(){
}

void Initialising::run(){
	//TODO
	ownerSequence->safeTransition("Initialised");
	cout << "Going to Initialised" << endl;
}