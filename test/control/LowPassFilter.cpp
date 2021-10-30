#include <eeros/control/filter/LowPassFilter.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>

#include <gtest/gtest.h>

#include <sstream>
#include <string>

using namespace eeros;
using namespace eeros::control;
using namespace math; 

TEST(controlLPFilterTest, templateInstantiations) {
  LowPassFilter<> lp1{0.01};
  LowPassFilter<int> lp2{0.2};
  LowPassFilter<Matrix<2,2>> lp3{0.1};
  EXPECT_TRUE(true);
}


TEST(controlLPFilterTest, doubleLPFilter) {
  LowPassFilter<> lp{0.5};
  Constant<> c1{2};
  lp.getIn().connect(c1.getOut());
  // this test case also tests if enable is true by default.
  EXPECT_TRUE(std::isnan(c1.getOut().getSignal().getValue()));
  c1.run();
  lp.run();
  EXPECT_DOUBLE_EQ(lp.getOut().getSignal().getValue(), 2);
  lp.run();
  EXPECT_DOUBLE_EQ(lp.getOut().getSignal().getValue(), 2);
  EXPECT_EQ(c1.getOut().getSignal().getTimestamp(), lp.getOut().getSignal().getTimestamp());
  c1.setValue(3);
  c1.run();
  lp.run();
  lp.run();
  lp.run();
  EXPECT_DOUBLE_EQ(lp.getOut().getSignal().getValue(), 2.875);
  EXPECT_EQ(c1.getOut().getSignal().getTimestamp(), lp.getOut().getSignal().getTimestamp());
}


TEST(controlLPFilterTest, enableDisable) {
  LowPassFilter<> lp{0.5};
  Constant<> c1{5};
  c1.run();
  lp.getIn().connect(c1.getOut());
  lp.disable();
  lp.run();
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue(), 5);
  c1.setValue(10);
  c1.run();
  lp.run();
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue(), 10);
  lp.enable();
  lp.run();
  // previous values will be stored regardless if filter is enabled or not.
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue(), 10);
  lp.run();
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue(), 10);
  EXPECT_EQ (c1.getOut().getSignal().getTimestamp(), lp.getOut().getSignal().getTimestamp());
}


TEST(controlLPFilterTest, vectorLPFilter) {
  LowPassFilter<Vector3> lp{0.5};
  Vector3 vec3{};
  vec3 << 1, 2, 3;
  Constant<Vector3> c1{vec3};
  c1.run();
  lp.getIn().connect(c1.getOut());
  lp.run();
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[0], 1);
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[1], 2);
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[2], 3);
  c1.setValue({2,3,4});
  c1.run();
  lp.run();
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[0], 1.5);
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[1], 2.5);
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[2], 3.5);
  lp.run();
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[0], 1.75);
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[1], 2.75);
  EXPECT_DOUBLE_EQ (lp.getOut().getSignal().getValue()[2], 3.75);
  EXPECT_EQ (c1.getOut().getSignal().getTimestamp(), lp.getOut().getSignal().getTimestamp());
}


TEST(controlLPFilterTest, printLPFilter) {
  LowPassFilter<> lp1{0.5};
  lp1.setName("my1stLPFilter");
    
  std::stringstream sstream{};
  sstream << lp1;
  std::string str1 = "Block LowPassFilter: 'my1stLPFilter' is enabled=1, alpha=0.5";
  std::string str2 = sstream.str();
  EXPECT_STREQ (str1.c_str(), str2.c_str());
}
