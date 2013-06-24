// Sequencer.cpp : Defines the entry point for the console application.
//

#include <eeros/core/Executor.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include "MySequence.hpp"

using namespace std;

//TODELETE
#include <iostream>

int main(int argc, char* argv[])
{
	TimeDomain timeDomain;
	MySequence mainSequence("MainSequence", &timeDomain);


	//Thread erzeugen:
	mainSequence.start();

	//Thread stoppen wird in Step Stopping gemacht
	//mainSequence.stop();
	
	while(!mainSequence.isTerminated()){
		cout << "waiting for executor to terminate..." << std::endl;
	}

	return 0;
}

