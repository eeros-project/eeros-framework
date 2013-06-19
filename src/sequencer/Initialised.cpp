
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Initialised.hpp>

Initialised::Initialised(Transitions* p_trans, string name, Sequence* owner):
SequencerStep(p_trans, name, owner)
{
}


Initialised::~Initialised(void)
{
}

void Initialised::run(){
	//TODO
	SequencerStep::run();
}