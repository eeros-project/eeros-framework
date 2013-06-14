

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Stopping.hpp>

//TODELETE
#include <iostream>

Stopping::Stopping(Transitions* p_trans, string name, Sequence* owner) :
SequencerStep(p_trans, name, owner)
{
}


Stopping::~Stopping(void)
{
}

void Stopping::run(){
	//TODO
	ownerSequence->safeTransition("");
	ownerSequence->stop();
	cout << "Stopping" << endl;
}
