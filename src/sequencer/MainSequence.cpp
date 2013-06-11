#include "StdAfx.h"

#include "eeros/core/Executor.hpp"

//TODO Pfad anpassen
#include "Sequence.hpp"
#include "SubSequence.hpp"

#include "MainSequence.hpp"


MainSequence::MainSequence(void)
{
}


MainSequence::~MainSequence(void)
{
	deleteSubSequences();
}

void MainSequence::deleteSubSequences(){
	list<SubSequence*>::reverse_iterator rev_iter = subSequences.rbegin();
	while(rev_iter != subSequences.rend()){
		delete (*rev_iter);
		rev_iter++;
	}
}

void MainSequence::addSubSequence(SubSequence* subSequence){
	subSequences.push_back(subSequence);
}
