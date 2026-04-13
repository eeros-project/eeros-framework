#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::logger;
using namespace eeros::math;

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();

  log.info() << "Matrix test started";

  Vector2 m1;
  log.info() << "m1=" << m1;
  Vector2 m2{2.0, 3.1};
  log.info() << "m2=" << m2;
  m2 = {-2.0, -3.1};
  log.info() << "m2=" << m2;
  m2 << -3.2, 1e3;
  log.info() << "m2=" << m2;
  // will trow an error due to wrong number of initializer elements
  // Vector2 m3{2.0};
  Vector2 m4(1.5);
  log.info() << "m4=" << m4;

  Vector2 v0{1.1,1.2}, v1{2.1,2.2};
  Matrix<2,1,Vector2> m10({v0,v1});
  log.info() << "m10=" << m10;

  Matrix<2,1,Vector2> m11{v0,v1};
  log.info() << "m11=" << m11;

  Matrix<2,1,Vector2> m12{{1.0,2.0},{2.1,-1}};
  log.info() << "m12=" << m12;

  Matrix<3, 3, int> m13{1, 4, 7, 2, 5, 8, 3, 6, 9}; 
  log.info() << "m13=" << m13;
  m13 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  log.info() << "m13=" << m13;

  Matrix<2, 2, int> m14;
  m14 = m13.getSubMatrix<2,2>(0,1);   // returns matrix [[2,5][3,6]]
  log.info() << "m14=" << m14;

  Vector2 m15{1.0, 2.0};
  m15 = m15 * 3.0;
  log.info() << "m15=" << m15;
}
