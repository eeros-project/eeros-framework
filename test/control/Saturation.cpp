#include <eeros/control/Saturation.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Constant.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test naming
TEST(controlSatTest, naming) {
  Saturation<> sat(0.3, 1.2);
  EXPECT_EQ(sat.getName(), std::string(""));
  sat.setName("saturation 1");
  EXPECT_EQ(sat.getName(), std::string("saturation 1"));
}

// Test initial values for NaN
TEST(controlSatTest, initialValue) {
  Saturation<Vector2> sat({-1,-2},{1,2});
  sat.setName("sat");
  EXPECT_TRUE(std::isnan(sat.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(sat.getOut().getSignal().getValue()[1]));
  try {
    sat.run();
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'sat'"));
  }
}

// Test function
TEST(controlSatTest, double) {
  Constant<> c0(1), c1(-1);
  Saturation<double> sat1(0, 0);
  sat1.getIn().connect(c0.getOut());
  c0.run(); c1.run(); sat1.run();
  EXPECT_EQ(sat1.getOut().getSignal().getValue(), 0);
  sat1.disable();
  sat1.run();
  EXPECT_EQ(sat1.getOut().getSignal().getValue(), 1);
  Saturation<double> sat2(-0.5, 0.5);
  sat2.getIn().connect(c0.getOut());
  sat2.run();
  EXPECT_EQ(sat2.getOut().getSignal().getValue(), 0.5);
  sat2.disable();
  sat2.run();
  EXPECT_EQ(sat2.getOut().getSignal().getValue(), 1);
  sat2.getIn().disconnect();
  sat2.getIn().connect(c1.getOut());
  sat2.run();
  EXPECT_EQ(sat2.getOut().getSignal().getValue(), -1);
  sat2.enable();
  sat2.run();
  EXPECT_EQ(sat2.getOut().getSignal().getValue(), -0.5);
  Saturation<double> sat3(-0.2, 0.3);
  sat3.getIn().connect(c0.getOut());
  sat3.run();
  EXPECT_EQ(sat3.getOut().getSignal().getValue(), 0.3);
  sat3.disable();
  sat3.run();
  EXPECT_EQ(sat3.getOut().getSignal().getValue(), 1);
  sat3.getIn().disconnect();
  sat3.getIn().connect(c1.getOut());
  sat3.run();
  EXPECT_EQ(sat3.getOut().getSignal().getValue(), -1);
  sat3.enable();
  sat3.run();
  EXPECT_EQ(sat3.getOut().getSignal().getValue(), -0.2);
}

// Test function
TEST(controlSatTest, vector) {
  Constant<Vector2> c0({1,-2.5});
  Saturation<Vector2> sat({-0.5,-0.8},{1.1, 0.5});
  sat.getIn().connect(c0.getOut());
  c0.run(); sat.run();
  EXPECT_EQ(sat.getOut().getSignal().getValue()[0], 1);
  EXPECT_EQ(sat.getOut().getSignal().getValue()[1], -0.8);
}


