#include "MyControlSystem.hpp"

using namespace eeros::control;

MyControlSystem::MyControlSystem() : 
	setpoint(0.0),
	setpointV(6),
	enc("q"),
	posController(174.5),
	speedController(565.48),
//	speedController(0),
	inertia(14.2e-7),
	invMotConst(1/15.7e-3 * 2.0),
	dac("dac"), executor(0.001) {
	
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
	
	executor.addRunnable(this);
}
	
void MyControlSystem::run() {
	setpoint.run();
//	setpointV.run();
//	diff2.run();
	enc.run();
	sum1.run();
	posController.run();
	diff1.run();
	sum2.run();
	speedController.run();
	inertia.run();
	invMotConst.run();
	dac.run();
}

void MyControlSystem::start() {
	executor.start();
}

void MyControlSystem::stop() {
	executor.stop();
}

MyControlSystem& MyControlSystem::instance() {
	static MyControlSystem controlSystemInstance;
	return controlSystemInstance;
}
