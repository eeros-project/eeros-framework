#include "StdAfx.h"

//TODO Pfad anpassen
#include "Sequence.hpp"
#include "Initialising.hpp"


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