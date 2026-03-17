#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Mul.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Delay.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Mux.hpp>
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

  log.info() << "Matrix test started";

  Vector2 m1;
  log.info() << "m1=" << m1;
  Vector2 m2{2.0, 3.1};
  log.info() << "m2=" << m2;
  // will trow an error due to wrong number of initializer elements
  // Vector2 m3{2.0};
  Vector2 m4(1.5);
  log.info() << "m4=" << m4;

  Vector2 v0{1.1,1.2}, v1{2.1,2.2};
  Matrix<2,1,Vector2> m5({v0,v1});
  log.info() << "m5=" << m5;

  Matrix<2,1,Vector2> m6{v0,v1};
  log.info() << "m6=" << m6;

  Matrix<2,1,Vector2> m7{{1,2},{2.1,-1}};
  log.info() << "m7=" << m7;

}

