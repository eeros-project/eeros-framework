#include "SimControlSystem.hpp"
#include <eeros/core/Executor.hpp>
#include <iostream>

using namespace eeros::control;

SimControlSystem::SimControlSystem(double ts) :
	simOut_in0("in0"),
	simOut_out0("out0"),
	aOut0("aOut0"),
	aInTest0("aInTest0"),
	aIn2("aIn2"),
	aOutTest2("aOutTest2"),
	setAOut0(-2.0),
	setAOutTest2(3.0),
	timedomain("Main time domain", ts, true)
{
	
	aOut0.getIn().connect(setAOut0.getOut());
	aOutTest2.getIn().connect(setAOutTest2.getOut());
  
	timedomain.addBlock(&simOut_out0);
	timedomain.addBlock(&simOut_in0);
	timedomain.addBlock(&setAOut0);
	timedomain.addBlock(&aOut0);
	timedomain.addBlock(&aInTest0);
	timedomain.addBlock(&aOutTest2);
	timedomain.addBlock(&setAOutTest2);
	timedomain.addBlock(&aIn2);
	
	
	eeros::task::Periodic td("control system",ts, timedomain);
	eeros::Executor::instance().add(timedomain);
}

SimControlSystem::~SimControlSystem(){
}

