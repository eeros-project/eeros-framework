#ifndef MOCK_ROBOT_CONTROL_SYSTEM_HPP_
#define MOCK_ROBOT_CONTROL_SYSTEM_HPP_

#include <eeros/control/I.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>

using namespace eeros::control;

class MockRobotControlSystem {
public:
	MockRobotControlSystem(double ts) : setpointX(0), setpointY(0), timedomain("Main time domain", ts, true) {
		iX.setInitCondition(0.3);
		iX.enable();
		iY.setInitCondition(-0.2);
		iY.enable();
		iX.getIn().connect(setpointX.getOut());
		iY.getIn().connect(setpointY.getOut());
		timedomain.addBlock(setpointX);
		timedomain.addBlock(setpointY);
		timedomain.addBlock(iX);
		timedomain.addBlock(iY);
		eeros::Executor::instance().add(timedomain);
	}
	~MockRobotControlSystem() { }
	
	Constant<> setpointX;
	Constant<> setpointY;
	I<> iX;
	I<> iY;

	TimeDomain timedomain;
};

#endif // MOCK_ROBOT_CONTROL_SYSTEM_HPP_
