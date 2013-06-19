
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Initialising.hpp>

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
	SequencerStep::run();
}