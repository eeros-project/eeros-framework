#ifndef MOCK_ROBOT_CONTROL_SYSTEM_HPP_
#define MOCK_ROBOT_CONTROL_SYSTEM_HPP_

#include <eeros/control/I.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/PathPlannerConstAcc.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

class MockRobotControlSystem {
 public:
  MockRobotControlSystem(double ts) : setpoint({0, 0}), pp({10,20}, {0.5,1.0}, {0.2,0.5}, ts), timedomain("Main time domain", ts, true) {
    i.setInitCondition({0.3, -0.2});
    i.enable();
    sw.switchToInput(0);
    i.getIn().connect(setpoint.getOut());
    sw.getIn(0).connect(i.getOut());
    sw.getIn(1).connect(pp.getPosOut());
    timedomain.addBlock(setpoint);
    timedomain.addBlock(i);
    timedomain.addBlock(pp);
    timedomain.addBlock(sw);
    Executor::instance().add(timedomain);
  }
  
  Constant<Matrix<2,1,double>> setpoint;
  I<Matrix<2,1,double>> i;
  Switch<2, Matrix<2,1,double>> sw;
  PathPlannerConstAcc<Matrix<2,1,double>> pp;

  TimeDomain timedomain;
};

#endif // MOCK_ROBOT_CONTROL_SYSTEM_HPP_
