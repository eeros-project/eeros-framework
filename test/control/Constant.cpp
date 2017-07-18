#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test initial values for NaN
TEST(controlConstantTest, initialValue) {
	Constant<> c1;
	Constant<int> c2;
	Constant<Matrix<2,1,double>> c3;
	Constant<Matrix<2,1,int>> c4;
		
	EXPECT_TRUE(std::isnan(c1.getOut().getSignal().getValue()));
	EXPECT_EQ(c2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
	EXPECT_TRUE(std::isnan(c3.getOut().getSignal().getValue()[0]));
	EXPECT_EQ(c4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
	EXPECT_EQ(c4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
	c1.run();
	c2.run();
	c3.run();
	c4.run();
	EXPECT_TRUE(std::isnan(c1.getOut().getSignal().getValue()));
	EXPECT_EQ(c2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
	EXPECT_TRUE(std::isnan(c3.getOut().getSignal().getValue()[0]));
	EXPECT_EQ(c4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
	EXPECT_EQ(c4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
}
