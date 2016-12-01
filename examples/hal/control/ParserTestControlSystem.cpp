#include "ParserTestControlSystem.hpp"
#include <eeros/core/Executor.hpp>
#include <iostream>

using namespace eeros::control;

ParserTestControlSystem::ParserTestControlSystem(double ts) :
	
	setPos(),
// 	readySig1("readySig1"),
	io1("io1"),
	ioOut("ioOut"),
	ioIn("ioIn"),
	dac1("dac1"),
	timedomain("Main time domain", ts, true)
{
	
	ioOut.getIn().connect(ioIn.getOut());
  
	timedomain.addBlock(&setPos);
// 	timedomain.addBlock(&readySig1);
	timedomain.addBlock(&io1);
	timedomain.addBlock(&ioIn);
	timedomain.addBlock(&ioOut);
	
	timedomain.addBlock(&dac1);
	
	eeros::task::Periodic td("control system",ts, timedomain);
	eeros::Executor::instance().add(timedomain);
}

ParserTestControlSystem::~ParserTestControlSystem(){
}

