#include <eeros/control/DeMux.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test naming
TEST(controlDeMuxTest, naming) {
  DeMux<2> dm;
  EXPECT_EQ(dm.getName(), std::string(""));
  dm.setName("demux 1");
  EXPECT_EQ(dm.getName(), std::string("demux 1"));
}

// Test initial values for NaN
TEST(controlDeMuxTest, initialValue) {
  DeMux<2> dm;
  dm.setName("dm");
  
  EXPECT_TRUE(std::isnan(dm.getOut(0).getSignal().getValue()));
  EXPECT_TRUE(std::isnan(dm.getOut(1).getSignal().getValue()));
  try {
    dm.run();
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'dm'"));
  }
}

// Test inexisting output
TEST(controlDeMuxTest, output) {
  DeMux<2,double> dm;
  dm.setName("demux");
  dm.getOut(1);
  try {
    dm.getOut(2);
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Trying to get inexistent element of output vector in block 'demux'"));
  }
}

// Test function
TEST(controlDeMuxTest, double) {
  Constant<Vector2> c0({1.1,1.2});
  DeMux<2,double> dm;
  dm.getIn().connect(c0.getOut());
  c0.run(); dm.run();
  EXPECT_TRUE(Utils::compareApprox(dm.getOut(0).getSignal().getValue(), 1.1, 0.0001));
  EXPECT_TRUE(Utils::compareApprox(dm.getOut(1).getSignal().getValue(), 1.2, 0.0001));
}

// Test function
TEST(controlDeMuxTest, vector) {
  Vector2 v0{1.1,1.2}, v1{2.1,2.2};
  Constant<Matrix<2,1,Vector2>> c0({v0,v1});
  DeMux<2,Vector2> dm;
  dm.getIn().connect(c0.getOut());
  c0.run(); dm.run();
  EXPECT_TRUE(Utils::compareApprox(dm.getOut(0).getSignal().getValue()[0], 1.1, 0.0001));
  EXPECT_TRUE(Utils::compareApprox(dm.getOut(0).getSignal().getValue()[1], 1.2, 0.0001));
  EXPECT_TRUE(Utils::compareApprox(dm.getOut(1).getSignal().getValue()[0], 2.1, 0.0001));
  EXPECT_TRUE(Utils::compareApprox(dm.getOut(1).getSignal().getValue()[1], 2.2, 0.0001));
}
