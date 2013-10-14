// Sequencer.cpp : Defines the entry point for the console application.
//
//TODELETE
#include <iostream>

#include <eeros/core/Executor.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/sequencer/Sequence.hpp>

#include "MySequencer.hpp"
#include "MySequence.hpp"

using namespace std;

int main(int argc, char* argv[])
{
	//The first allocation of a sequencer is automatically the Main-Sequencer!!
	MySequencer mainSequencer("MainSequencer");
	MySequence sequence("MySequence", mainSequencer);

	//Thread erzeugen:
	mainSequencer.start();

	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		cout << "waiting for executor to terminate..." << std::endl;
	}

	return 0;
}

