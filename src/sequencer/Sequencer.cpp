// Sequencer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "eeros/core/Executor_PARV.hpp"

//TODO Pfad anpassen
#include "MySequence.hpp"

using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	//TODO: sollte ohne Periode sein
	MySequence mainSequence(1.0, "MainSequence");
	mainSequence.fillSequencerSteps();


	//Thread erzeugen:
	mainSequence.start();

	//Thread stoppen wird in Step Stopping gemacht
	//mainSequence.stop();
	
	while(!mainSequence.isTerminated()){
		cout << "waiting for executor to terminate..." << std::endl;
	}


	return 0;
}

