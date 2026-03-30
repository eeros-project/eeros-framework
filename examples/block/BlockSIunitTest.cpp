#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/SocketData.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Mul.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Delay.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/MouseInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/can/CANopenReceive.hpp>
#include <eeros/control/Blockio.hpp>

using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::siunit;
using namespace eeros;

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
//  log.set(&w);

  log.info() << "Block test started";

// Gain<Matrix<2,2>,Matrix<2,2>,true> g9{};
  // Constant<Vector2> c1(0.2);
  // c1.setName("constant 1");
  // c1.getOut().getSignal().setName("signal 1");
  // c1.run();
  // log.info() << c1 << ": output = " << c1.getOut().getSignal();
  //
  // constexpr std::array<SIUnit, 2> u10{ Meter, MeterPerSecond };
  // Constant<Vector2, Meter> c10(0.2);
  // c10.setName("constant 10");
  // c10.getOut().getSignal().setName("signal 10");
  // c10.run();
  // log.info() << c10 << ": output = " << c10.getOut().getSignal();

  // constexpr std::array<SIUnit, 2> u11 { Meter, Meter };
  // Mul<double,double,double,u11,Meter> sum;
  // sum.setName("summation");
  // sum.getOut().getSignal().setName("signal summation");
  // sum.getIn<0>().connect(c1.getOut());
  // sum.getIn(1).connect(step.getOut());
  // sum.run();
  // log.info() << sum << ":output = " << sum.getOut().getSignal();

  // constexpr std::array<SIUnit, 1> u1 { Meter };
  // Blockio<1,1,double,double,u1,u1> gen([&](){
  //   gen.getOut().getSignal().setValue(gen.getIn().getSignal().getValue() * 2);
  //   gen.getOut().getSignal().setTimestamp(gen.getIn().getSignal().getTimestamp());
  // });
  // gen.setName("generic block");
  // gen.getIn().connect(c1.getOut());
  // gen.getIn().connect(c1.getOut());
  // gen.getOut().getSignal().setName("output");
  // gen.run();
  // log.info() << gen << ": output = " << gen.getOut().getSignal();
  //
  //
  // Vector3 v1{3,4,5};
  // log.info() << v1;
  // Constant<Vector3,Meter> c2(v1);
  // c2.setName("constant 2");
  // c2.getOut().getSignal().setName("signal 2");
  // c2.run();
  // log.info() << c2 << ": output = " << c2.getOut().getSignal();
  //
  // Constant<Matrix<3,1>,Meter> c3({1.2, 2.5, 3});
  // c3.setName("constant 3");
  // c3.getOut().getSignal().setName("signal 3");
  // c3.run();
  // log.info() << c3 << ": output = " << c3.getOut().getSignal();
  //
  // Step<Vector3,Meter> step(1.5, -3.14159265359, 5);
  // step.setName("step");
  // step.getOut().getSignal().setName("signal step");
  // step.run();
  // log.info() << step << ": output = " << step.getOut().getSignal();
  //
  // Step<Vector3,Meter> step2({2, 2.5, 3}, -3.14159265359, 5);
  // step2.setName("step 2");
  // log.info() << step2;

  // constexpr std::array<SIUnit, 2> u2 { Meter, Meter };
  // Sum<2,Vector3,Meter,Meter> sum;
  // sum.setName("summation");
  // sum.getOut().getSignal().setName("signal summation");
  // sum.getIn(0).connect(c2.getOut());
  // sum.getIn(1).connect(step.getOut());
  // sum.run();
  // log.info() << sum << ":output = " << sum.getOut().getSignal();

  // constexpr std::array<SIUnit, 2> units { Volt, Watt };
  // Blockio<2, 0, double, double, units> block{};
  // Input<double, Volt>& input = block.getIn<0>();
  // Input<double, Watt>& input2 = block.getIn<1>();
  // Output<double, Volt> out;
  // Output<double, Watt> out2;
  // block.getIn<0>().connect(out);
  // block.getIn<1>().connect(out2);
  Constant<double, Volt> c1(1.5);
  SocketData<double, Vector2, Volt, Ampere> sock1("127.0.0.1", 9876, 0.1);
  sock1.getIn().connect(c1.getOut());
  Sum<3, Vector2, Ampere> sum1;
  sum1.getIn(0).connect(sock1.getOut());
  MouseInput<Metre> mouse1("/dev/mouse");
  // MouseInput<> mouse1("/dev/mouse");
  Sum<2, Vector4, Metre> sum2;
  sum2.getIn(1).connect(mouse1.getOut());
  PeripheralOutput<bool> out1("out1");
  // PeripheralOutput<bool> out1("out1");
  DeMux<3,bool> demux1;
  demux1.getIn().connect(mouse1.getButtonOut());
  out1.getIn().connect(demux1.getOut(0));
  log.info() << mouse1.getButtonOut().getSignal();

  constexpr std::array<SIUnit, 2> units { Volt, Watt };
  MouseInput<Watt> mouse2("/dev/mouse");

  // CANopen co = std::make_shared<CAN::CANSocket>("can0");
  CANopenReceive<> canReceive({1, 2});
  Sum<3, double> sum3;
  sum3.getIn(0).connect(canReceive.getOut());


  return 0;
}

