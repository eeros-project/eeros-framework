#include "StdAfx.h"

//TODO Pfad anpassen
#include "Sequence.hpp"
#include "Initialised.hpp"


Initialised::Initialised(Transitions* p_trans, string name, Sequence* owner):
SequencerStep(p_trans, name, owner)
{
}


Initialised::~Initialised(void)
{
}

void Initialised::run(){
	//TODO
	ownerSequence->safeTransition("Homed");
	cout << "Going to Homed" << endl;
}