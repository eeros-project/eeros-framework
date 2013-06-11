#include "StdAfx.h"

//TODO Pfad anpassen
#include "Sequence.hpp"
#include "Stopping.hpp"


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
