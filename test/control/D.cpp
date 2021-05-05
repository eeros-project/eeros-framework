#include <eeros/control/D.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test naming
TEST(controlDTest, naming) {
  D<> d1;
  EXPECT_EQ(d1.getName(), std::string(""));
  d1.setName("differentiator 1");
  EXPECT_EQ(d1.getName(), std::string("differentiator 1"));
}

// Test initial values for NaN
TEST(controlDTest, initialValue) {
  D<> d1;
  D<int> d2;
  D<Matrix<2,1,double>> d3;
  D<Matrix<2,1,int>> d4;
  d1.setName("d1");
  d2.setName("d2");
  d3.setName("d3");
  d4.setName("d4");
    
  EXPECT_TRUE(std::isnan(d1.getOut().getSignal().getValue()));
  EXPECT_EQ(d2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
  EXPECT_TRUE(std::isnan(d3.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(d3.getOut().getSignal().getValue()[1]));
  EXPECT_EQ(d4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
  EXPECT_EQ(d4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
  try {
    d1.run();
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'd1'"));
  }
  try {
    d2.run();
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'd2'"));
  }
  try {
    d3.run();
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'd3'"));
  }
  try {
    d4.run();
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'd4'"));
  }
}

// test function
TEST(controlDTest, running) {
  Constant<> c1(1.0);
  D<> d1;
  d1.getIn().connect(c1.getOut());
  c1.run();
  d1.run();
  EXPECT_TRUE(std::isnan(d1.getOut().getSignal().getValue()));
  c1.run();
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue(), 0, 1e-10));
  c1.run();
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue(), 0, 1e-10));
  timestamp_t start = d1.getOut().getSignal().getTimestamp();
  usleep(10000);
  c1.setValue(0);
  c1.run();
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue(), -100, 50));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getTimestamp() - start, 5000000, 1000000));
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue(), -100, 50));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getTimestamp() - start, 5000000, 1000000));
  c1.run();
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue(), 0, 0.01));
}

// test function
TEST(controlDTest, running2) {
  Constant<Matrix<2,1,double>> c1(1.0);
  D<Matrix<2,1,double>> d1;
  d1.getIn().connect(c1.getOut());
  c1.run();
  d1.run();
  EXPECT_TRUE(std::isnan(d1.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(d1.getOut().getSignal().getValue()[1]));
  c1.run();
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[0], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[1], 0, 1e-10));
  c1.run();
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[0], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[1], 0, 1e-10));
  timestamp_t start = d1.getOut().getSignal().getTimestamp();
  usleep(10000);
  c1.setValue(0.0);
  c1.run();
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[0], -100, 50));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[1], -100, 50));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getTimestamp() - start, 5000000, 1000000));
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[0], -100, 50));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[1], -100, 50));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getTimestamp() - start, 5000000, 1000000));
  c1.run();
  d1.run();
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[0], 0, 0.01));
  EXPECT_TRUE(Utils::compareApprox(d1.getOut().getSignal().getValue()[1], 0, 0.01));
}
