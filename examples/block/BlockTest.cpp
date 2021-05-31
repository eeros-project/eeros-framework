#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Blockio.hpp>

using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::math;

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
//  log.set(&w);
 
  log.info() << "Block test started";
  
  Constant<> c1(0.2);
  c1.setName("constant 1");
  c1.getOut().getSignal().setName("signal 1");
  c1.run();
  log.info() << c1 << ": output = " << c1.getOut().getSignal();
  
  Blockio<1,1> gen([&](){
    gen.getOut().getSignal().setValue(gen.getIn().getSignal().getValue() * 2);
    gen.getOut().getSignal().setTimestamp(gen.getIn().getSignal().getTimestamp());
  });
  gen.setName("generic block");
  gen.getIn().connect(c1.getOut());
  gen.getIn().connect(c1.getOut());
  gen.run();
  log.info() << gen << ": output = " << gen.getOut().getSignal();
//   log.info() <<  ": output = " << gen.getOut().getSignal();
  
  
  Vector3 v1{3,4,5};
  log.info() << v1;
  Constant<Vector3> c2(v1);	
  c2.setName("constant 2");
  c2.getOut().getSignal().setName("signal 2");
  c2.run();
  log.info() << c2 << ": output = " << c2.getOut().getSignal();
  
  Constant<Matrix<3,1>> c3({1.2, 2.5, 3});
  c3.setName("constant 3");
  c3.getOut().getSignal().setName("signal 3");
  c3.run();
  log.info() << c3 << ": output = " << c3.getOut().getSignal();
  
  Step<Vector3> step(1.5, -3.14159265359, 5);
  step.setName("step");
  step.getOut().getSignal().setName("signal step");
  step.run();
  log.info() << step << ": output = " << step.getOut().getSignal();

  Step<Vector3> step2({2, 2.5, 3}, -3.14159265359, 5);	
  step2.setName("step 2");
  log.info() << step2;
  
  Sum<2,Vector3> sum;
  sum.setName("summation");
  sum.getOut().getSignal().setName("signal summation");
  sum.getIn(0).connect(c2.getOut());
  sum.getIn(1).connect(step.getOut());
  sum.run();
  log.info() << sum << ":output = " << sum.getOut().getSignal();

  Constant<Matrix<1,1>> c4(6);		
  Constant<Matrix<2,2>> c5(2.7);		
  Constant<Matrix<2,2>> c6({2.7, -1.2, 0, 0});
  c4.run(); 
  c5.run();
  c6.run();
  log.info() << c4.getOut().getSignal();
  log.info() << c5.getOut().getSignal();
  log.info() << c6.getOut().getSignal();

  return 0;
}
