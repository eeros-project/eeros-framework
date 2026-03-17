#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test initial values for NaN
TEST(controlConstantTest, initialNan) {
  Constant<> c1;
  Constant<int> c2;
  Constant<Matrix<2,1,double>,siunit::Volt> c3;
  Constant<Matrix<2,1,int>> c4;
  Constant<double,siunit::Ampere> c5(3.5);

  // values are set to default values
  EXPECT_TRUE(std::isnan(c1.getOut().getSignal().getValue()));
  EXPECT_EQ(c2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
  EXPECT_TRUE(std::isnan(c3.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(c3.getOut().getSignal().getValue()[1]));
  EXPECT_EQ(c4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
  EXPECT_EQ(c4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
  EXPECT_TRUE(std::isnan(c5.getOut().getSignal().getValue()));
  c1.run();
  c2.run();
  c3.run();
  c4.run();
  c5.run();
  EXPECT_TRUE(std::isnan(c1.getOut().getSignal().getValue()));
  EXPECT_EQ(c2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
  EXPECT_TRUE(std::isnan(c3.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(c3.getOut().getSignal().getValue()[1]));
  EXPECT_EQ(c4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
  EXPECT_EQ(c4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
  EXPECT_EQ(c5.getOut().getSignal().getValue(), 3.5);
}

// Test initial values
TEST(controlConstantTest, initValue) {
  Constant<> c1(2.5);
  c1.run();
  EXPECT_EQ(c1.getOut().getSignal().getValue(), 2.5);
  Constant<> c2(3);
  c2.run();
  EXPECT_EQ(c2.getOut().getSignal().getValue(), 3);
  Constant<> c3{-2.5};
  c3.run();
  EXPECT_EQ(c3.getOut().getSignal().getValue(), -2.5);
  Constant<int> c4{-2};
  c4.run();
  EXPECT_EQ(c4.getOut().getSignal().getValue(), -2);

  Constant<Vector2> c10({2.5, 12.5});
  c10.run();
  EXPECT_EQ(c10.getOut().getSignal().getValue()[0], 2.5);
  EXPECT_EQ(c10.getOut().getSignal().getValue()[1], 12.5);
  Constant<Matrix<2,1,int>> c11({3, 2});
  c11.run();
  EXPECT_EQ(c11.getOut().getSignal().getValue()[0], 3);
  EXPECT_EQ(c11.getOut().getSignal().getValue()[1], 2);
  Constant<Vector2> c12{2.5, 12.5};
  c12.run();
  EXPECT_EQ(c12.getOut().getSignal().getValue()[0], 2.5);
  EXPECT_EQ(c12.getOut().getSignal().getValue()[1], 12.5);
  Constant<Vector2> c13{2.5, 12.5};
  c12.run();
  EXPECT_EQ(c12.getOut().getSignal().getValue()[0], 2.5);
  EXPECT_EQ(c12.getOut().getSignal().getValue()[1], 12.5);

  Vector2 v0{1.1,1.2}, v1{2.1,2.2};
  Constant<Matrix<2,1,Vector2>> c20({v0,v1});
  Matrix<2,1,Vector2> m1{v0,v1};
  Constant<Matrix<2,1,Vector2>> c21(m1);
  // Constant<Matrix<2,1,Vector2>> c22({{1.0,2.0},{3.0,4.0}});

}

// Test set values
TEST(controlConstantTest, setValue) {
  Constant<> c1(2.5);
  c1.run();
  EXPECT_EQ(c1.getOut().getSignal().getValue(), 2.5);
  c1.setValue(-0.2);
  EXPECT_EQ(c1.getOut().getSignal().getValue(), 2.5);
  c1.run();
  EXPECT_EQ(c1.getOut().getSignal().getValue(), -0.2);
  Constant<Vector2> c2({1.1, 2.5});
  c2.run();
  EXPECT_EQ(c2.getOut().getSignal().getValue()[0], 1.1);
  EXPECT_EQ(c2.getOut().getSignal().getValue()[1], 2.5);
  c2.setValue({-0.2, 0.1});
  EXPECT_EQ(c2.getOut().getSignal().getValue()[0], 1.1);
  EXPECT_EQ(c2.getOut().getSignal().getValue()[1], 2.5);
  c2.run();
  EXPECT_EQ(c2.getOut().getSignal().getValue()[0], -0.2);
  EXPECT_EQ(c2.getOut().getSignal().getValue()[1], 0.1);
}

TEST(controlConstantTest, print) {
  Constant<double,siunit::Metre> c{5.2};
  c.setName("c1");
  c.run();
  std::stringstream sstream{};
  sstream << c;
  std::string str1 = "Block constant: 'c1' current val = 5.2";
  std::string str2 = sstream.str();
  EXPECT_STREQ (str1.c_str(), str2.c_str());
}


