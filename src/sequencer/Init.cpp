#include "StdAfx.h"

//TODO Pfad anpassen
#include "Sequence.hpp"
#include "Init.hpp"


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
