#include <eeros/control/WrapAround.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test template instantiations
TEST(controlWrapAroundTest, templateInstantiations) {
  WrapAround<> w1{0, 1};
  WrapAround<int> w2{0, 1};
  WrapAround<> w3{-0.1, 0.1};
  WrapAround<int> w4{2, 4};
  WrapAround<Matrix<2,1,double>, double> w5{1.1,2.4};
  WrapAround<Matrix<2,1,double>> w6{{1,1}, {2,2}};
  EXPECT_TRUE(true);
}

// Test initial values for NaN
TEST(controlWrapAroundTest, initialValue) {
  WrapAround<> w1(0,1);
  WrapAround<int> w2(0,1);
  WrapAround<Vector2> w3({1,1}, {2,2});
  WrapAround<Matrix<2,1,int>> w4{{1,1}, {2,2}};
  w1.setName("w1");
  std::stringstream sstream{};
  sstream << w1;
  std::string str1 = "Block WrapAround: 'w1' is enabled=1, minVal=0, maxVal=1";
  std::string str2 = sstream.str();
  EXPECT_STREQ (str1.c_str(), str2.c_str());
    
  EXPECT_TRUE(std::isnan(w1.getOut().getSignal().getValue()));
  EXPECT_EQ(w2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
  EXPECT_TRUE(std::isnan(w3.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(w3.getOut().getSignal().getValue()[1]));
  EXPECT_EQ(w4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
  EXPECT_EQ(w4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
}

// 
TEST(controlWrapAroundTest, run1) {
  Constant<> c1(2.0);
  WrapAround<> w(1.5, 3.5);
  w.getIn().connect(c1.getOut());
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), 2.0);
  c1.setValue(4.5);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), 2.5);
  c1.setValue(1.0);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), 3.0);
  c1.setValue(1.5);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), 1.5);
  c1.setValue(3.5);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), 3.5);
}

TEST(controlWrapAroundTest, run2) {
  Constant<> c1(0);
  WrapAround<> w(-1.0, 1.0);
  w.getIn().connect(c1.getOut());
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), 0);
  c1.setValue(1.5);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), -0.5);
  c1.setValue(-1.5);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), 0.5);
}

TEST(controlWrapAroundTest, run3) {
  Constant<> c1(-0.9);
  WrapAround<> w(-2.0, -1.0);
  w.getIn().connect(c1.getOut());
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), -1.9);
  c1.setValue(-1.5);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), -1.5);
  c1.setValue(-2.1);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue(), -1.1);
}

TEST(controlWrapAroundTest, run4) {
  Constant<Vector2> c1({2,2});
  WrapAround<Vector2> w(1.0, 3.0);
  w.getIn().connect(c1.getOut());
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue()[0], 2);
  EXPECT_EQ(w.getOut().getSignal().getValue()[1], 2);
  c1.setValue(3.1);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue()[0], 1.1);
  EXPECT_EQ(w.getOut().getSignal().getValue()[1], 1.1);
  c1.setValue(-0.1);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue()[0], 1.9);
  EXPECT_EQ(w.getOut().getSignal().getValue()[1], 1.9);
}

TEST(controlWrapAroundTest, run5) {
  Constant<Vector2> c1({2,2});
  WrapAround<Vector2> w({1.0,1.5}, {3.0, 3.5});
  w.getIn().connect(c1.getOut());
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue()[0], 2);
  EXPECT_EQ(w.getOut().getSignal().getValue()[1], 2);
  c1.setValue(3.1);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue()[0], 1.1);
  EXPECT_EQ(w.getOut().getSignal().getValue()[1], 3.1);
  c1.setValue(4.0);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue()[0], 2.0);
  EXPECT_EQ(w.getOut().getSignal().getValue()[1], 2.0);
  c1.setValue(-0.1);
  c1.run();
  w.run();
  EXPECT_EQ(w.getOut().getSignal().getValue()[0], 1.9);
  EXPECT_EQ(w.getOut().getSignal().getValue()[1], 1.9);
}
