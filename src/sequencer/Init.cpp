
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Init.hpp>

//TODELETE
#include <iostream>


Init::Init(Transitions* p_trans, string name, Sequence* owner) :
SequencerStep(p_trans, name, owner)
{
}


Init::~Init(void)
{
}

void Init::run(){
	ownerSequence->safeTransition("Initialising");
	//für TEst
	cout << "Going to Initialising" << endl;
}
