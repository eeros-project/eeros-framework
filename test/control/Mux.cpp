#include <eeros/control/Mux.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Constant.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test naming
TEST(controlMuxTest, naming) {
  Mux<2> m;
  EXPECT_EQ(m.getName(), std::string(""));
  m.setName("mux 1");
  EXPECT_EQ(m.getName(), std::string("mux 1"));
}

// Test initial values for NaN
TEST(controlMuxTest, initialValue) {
  Mux<2,double> m;
  m.setName("mux");
  EXPECT_TRUE(std::isnan(m.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(m.getOut().getSignal().getValue()[1]));
  try {
    m.run();
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'mux'"));
  }
}

// Test inexisting input
TEST(controlMuxTest, input) {
  Mux<2,double> m;
  m.setName("mux");
  m.getIn(1);
  try {
    m.getIn(2);
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Trying to get inexistent input in block 'mux'"));
  }
}

// Test function
TEST(controlMuxTest, double) {
  Constant<> c0(1), c1(2);
  Mux<2,double> m;
  m.getIn(0).connect(c0.getOut());
  m.getIn(1).connect(c1.getOut());
  c0.run(); c1.run(); m.run();
  EXPECT_TRUE(Utils::compareApprox(m.getOut().getSignal().getValue()[0], 1, 0.0001));
  EXPECT_TRUE(Utils::compareApprox(m.getOut().getSignal().getValue()[1], 2, 0.0001));
}

// Test function
TEST(controlMuxTest, vector) {
  Constant<Vector2> c0({1.1,1.2}), c1({2.1,2.2});
  Mux<2,Vector2> m;
  m.getIn(0).connect(c0.getOut());
  m.getIn(1).connect(c1.getOut());
  c0.run(); c1.run(); m.run();
  EXPECT_TRUE(Utils::compareApprox(m.getOut().getSignal().getValue()[0][0], 1.1, 0.0001));
  EXPECT_TRUE(Utils::compareApprox(m.getOut().getSignal().getValue()[0][1], 1.2, 0.0001));
  EXPECT_TRUE(Utils::compareApprox(m.getOut().getSignal().getValue()[1][0], 2.1, 0.0001));
  EXPECT_TRUE(Utils::compareApprox(m.getOut().getSignal().getValue()[1][1], 2.2, 0.0001));
}


