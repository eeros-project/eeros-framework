#include "MyControlSystem.hpp"

using namespace eeros::control;

MyControlSystem::MyControlSystem() : setpoint(0.0), enc("enc"), posController(174.5), sum2(3), speedController(565.48), inertia(14.2e-7), invMotConst(1/15.7e-3 * 2.0), dac("dac"), executor(0.001) {
	
	setpoint.getOut().setName("phi_desired");
	setpoint.getOut().setUnit("rad");

	enc.getOut().setName("phi_actual");
	enc.getOut().setUnit("rad");

	diff1.getOut().setName("phi_d_actual");
	diff1.getOut().setUnit("rad/s");
	sum1.negateInput(1);
	sum1.getOut().setName("phi_e");
	sum1.getOut().setUnit("rad");
	
	posController.getOut().setName("phi_d_set");
	posController.getOut().setUnit("rad/s");
	
	diff2.getOut().setName("phi_d_set_ff");
	diff2.getOut().setUnit("rad/s");
	
	sum2.negateInput(1);
	sum2.getOut().setName("phi_d_e");
	sum2.getOut().setUnit("rad/s");
	
	speedController.getOut().setName("phi_dd_set");
	speedController.getOut().setUnit("rad/s^2");

	inertia.getOut().setName("M");
	inertia.getOut().setUnit("Nm");

	invMotConst.getOut().setName("i");
	invMotConst.getOut().setUnit("A");
	
	diff1.getIn().connect(enc.getOut());
	sum1.getIn(0).connect(setpoint.getOut());
	sum1.getIn(1).connect(enc.getOut());
	posController.getIn().connect(sum1.getOut());
	diff2.getIn().connect(setpoint.getOut());
	sum2.getIn(0).connect(posController.getOut());
	sum2.getIn(1).connect(diff1.getOut());
	sum2.getIn(2).connect(diff2.getOut());
	speedController.getIn().connect(sum2.getOut());
	inertia.getIn().connect(speedController.getOut());
	invMotConst.getIn().connect(inertia.getOut());
	dac.getIn().connect(invMotConst.getOut());
	
	executor.addRunnable(this);
}
	
void MyControlSystem::run() {
	setpoint.run();
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
