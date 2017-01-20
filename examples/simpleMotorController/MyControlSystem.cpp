#include "MyControlSystem.hpp"
#include <eeros/core/Executor.hpp>

using namespace eeros::control;

MyControlSystem::MyControlSystem(double ts) : 
	setpoint(0.0),
	setpointV(6),
	enc("q"),
// 	posController(174.5),
	posController(10),
// 	speedController(565.48),
	speedController(100),
	inertia(14.2e-7),
	invMotConst(1/15.7e-3 * 2.0),
	dac("dac"),
	timedomain("Main time domain", ts, true) {
	
	setpoint.getOut().getSignal().setName("phi_desired");

	enc.getOut().getSignal().setName("phi_actual");

	diff1.getOut().getSignal().setName("phi_d_actual");
	sum1.negateInput(1);
	sum1.getOut().getSignal().setName("phi_e");
	
	posController.getOut().getSignal().setName("phi_d_set");
	
	diff2.getOut().getSignal().setName("phi_d_set_ff");
	
	sum2.negateInput(1);
	sum2.getOut().getSignal().setName("phi_d_e");
	
	speedController.getOut().getSignal().setName("phi_dd_set");

	inertia.getOut().getSignal().setName("M");

	invMotConst.getOut().getSignal().setName("i");
	
	diff1.getIn().connect(enc.getOut());
	sum1.getIn(0).connect(setpoint.getOut());
	sum1.getIn(1).connect(enc.getOut());
	posController.getIn().connect(sum1.getOut());
	diff2.getIn().connect(setpoint.getOut());
	sum2.getIn(0).connect(posController.getOut());
// 	sum2.getIn(0).connect(setpointV.getOut());
	sum2.getIn(1).connect(diff1.getOut());
	sum2.getIn(2).connect(diff2.getOut());
	speedController.getIn().connect(sum2.getOut());
	inertia.getIn().connect(speedController.getOut());
	invMotConst.getIn().connect(inertia.getOut());
	dac.getIn().connect(invMotConst.getOut());
// 	dac.getIn().connect(setpointV.getOut());
	
	timedomain.addBlock(&setpoint);
// 	timedomain.addBlock(&setpointV);
//	timedomain.addBlock(&diff2);
	timedomain.addBlock(&enc);
	timedomain.addBlock(&sum1);
	timedomain.addBlock(&posController);
	timedomain.addBlock(&sum2);
	timedomain.addBlock(&speedController);
	timedomain.addBlock(&inertia);
	timedomain.addBlock(&invMotConst);
	timedomain.addBlock(&dac);

	eeros::task::Periodic td("control system",ts, timedomain);
	eeros::Executor::instance().add(timedomain);
}

MyControlSystem::~MyControlSystem(){
	
}

// void MyControlSystem::start() {
// 	timedomain.start();
// }
// 
// void MyControlSystem::stop() {
// 	timedomain.stop();
// }
