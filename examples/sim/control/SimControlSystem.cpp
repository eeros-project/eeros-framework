#include "SimControlSystem.hpp"
#include <eeros/core/Executor.hpp>
#include <iostream>

using namespace eeros::control;

SimControlSystem::SimControlSystem(double ts) :
	simOut_in0("in0"),
	simOut_out0("out0"),
	timedomain("Main time domain", ts, true)
{
	
// 	ioOut.getIn().connect(ioIn.getOut());
  
	timedomain.addBlock(&simOut_out0);
	timedomain.addBlock(&simOut_in0);
	
	eeros::task::Periodic td("control system",ts, timedomain);
	eeros::Executor::instance().add(timedomain);
}

SimControlSystem::~SimControlSystem(){
}

