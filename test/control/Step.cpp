#include <eeros/control/Step.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test template instantiations
TEST(controlStepTest, templateInstantiations) {
  Step<> g1{};
  Step<int> g2{};
  Step<> g3{1.0, 2.0, 2.0};
  Step<int> g4{1, 10, 2.0};
  Step<Matrix<2,1,double>> g5{};
  Step<Matrix<2,1,double>> g6{2.5, 1.2, 0.5};
  Step<Matrix<2,1,int>> g7{2, 1, 0.5};

  EXPECT_TRUE(true);
}

//Test initial values for NaN
TEST(controlStepTest, initialValue) {
  Step<> s1;
  Step<int> s2;
  Step<Vector2> s3(1.0, 1.0, 1.0);
  Step<Matrix<2,1,int>> s4{1, 1, 1.0};
  s1.setName("s1");
  s2.setName("s2");
  s3.setName("s3");
  s4.setName("s4");
    
  EXPECT_TRUE(std::isnan(s1.getOut().getSignal().getValue()));
  EXPECT_EQ(s2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
  EXPECT_TRUE(std::isnan(s3.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(s3.getOut().getSignal().getValue()[1]));
  EXPECT_EQ(s4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
  EXPECT_EQ(s4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
}

//Test initial values
TEST(controlStepTest, printStep) {
  Step<> s(0.5, 1.2, 0.1);
  s.setName("step");
  std::stringstream sstream{};
  sstream << s;
  std::string str1 = "Block step: 'step' init val = 0.5, step height = 1.2, delay = 0.1";
  std::string str2 = sstream.str();
  EXPECT_STREQ (str1.c_str(), str2.c_str());
  s.run();
  sstream.str("");
  sstream << s;
  str2 = sstream.str();
  EXPECT_STREQ (str1.c_str(), str2.c_str());
}

// 
TEST(controlStepTest, run) {
  Step<> s(0.5, 1.0, 0.1);
  s.run();
  EXPECT_EQ(s.getOut().getSignal().getValue(), 0.5);
  s.run();
  EXPECT_EQ(s.getOut().getSignal().getValue(), 0.5);
  usleep(100000);
  s.run();
  EXPECT_EQ(s.getOut().getSignal().getValue(), 1.5);
}