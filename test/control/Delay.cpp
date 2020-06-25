#include <eeros/control/Delay.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test initial values for NaN
TEST(controlDelayTest, initialValue) {
  Delay<> del(1.5, 0.5);
  del.setName("delay");
  
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()));
  try {
    del.run();
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'delay'"));
  }
}

// Test buflen
TEST(controlDelayTest, buflen) {
  try {
    Delay<> del(1.5, 5);
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("delay has negative or zero length"));
  }
}

// Test delay
TEST(controlDelayTest, delay) {
  Delay<> del(1.5, 0.5);
  Constant<> c(0.2);
  del.getIn().connect(c.getOut());
  c.run();
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()));
  EXPECT_EQ(del.getOut().getSignal().getTimestamp(), 0);
  del.run();
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()));
  del.run();
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()));
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue(), 0.2);
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue(), 0.2);
  c.setValue(-0.3);
  c.run();
  EXPECT_EQ(del.getOut().getSignal().getValue(), 0.2);
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue(), 0.2);
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue(), 0.2);
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue(), -0.3);
}

// Test delay vector
TEST(controlDelayTest, delayVector) {
  Delay<Vector2> del(1.5, 0.5);
  Constant<Vector2> c({10, 20});
  del.getIn().connect(c.getOut());
  c.run();
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()[1]));
  EXPECT_EQ(del.getOut().getSignal().getTimestamp(), 0);
  del.run();
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()[1]));
  del.run();
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(del.getOut().getSignal().getValue()[1]));
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue()[0], 10);
  EXPECT_EQ(del.getOut().getSignal().getValue()[1], 20);
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue()[0], 10);
  EXPECT_EQ(del.getOut().getSignal().getValue()[1], 20);
  c.setValue({5.6, -23});
  c.run();
  EXPECT_EQ(del.getOut().getSignal().getValue()[0], 10);
  EXPECT_EQ(del.getOut().getSignal().getValue()[1], 20);
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue()[0], 10);
  EXPECT_EQ(del.getOut().getSignal().getValue()[1], 20);
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue()[0], 10);
  EXPECT_EQ(del.getOut().getSignal().getValue()[1], 20);
  del.run();
  EXPECT_EQ(del.getOut().getSignal().getValue()[0], 5.6);
  EXPECT_EQ(del.getOut().getSignal().getValue()[1], -23);
}
