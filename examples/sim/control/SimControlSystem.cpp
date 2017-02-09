#include "SimControlSystem.hpp"
#include <eeros/core/Executor.hpp>
#include <iostream>

using namespace eeros::control;

SimControlSystem::SimControlSystem(double ts) :
	in0("in0"),
	out0("out0"),
	timedomain("Main time domain", ts, true)
{
	
// 	ioOut.getIn().connect(ioIn.getOut());
  
	timedomain.addBlock(&out0);
	timedomain.addBlock(&in0);
	
	eeros::task::Periodic td("control system",ts, timedomain);
	eeros::Executor::instance().add(timedomain);
}

SimControlSystem::~SimControlSystem(){
}

