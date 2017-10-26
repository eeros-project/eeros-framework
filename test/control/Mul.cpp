#include <eeros/control/Mul.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test initial values for NaN
TEST(controlMulTest, initialValue) {
	Mul<> m1;
	Mul<int,int,int> m2;
	Mul<Matrix<2,2,double>,Matrix<2,1,double>,Matrix<2,1,double>> m3;
	Mul<Matrix<2,2,int>,Matrix<2,1,int>,Matrix<2,1,int>> m4;
	m1.setName("m1");
	m2.setName("m2");
	m3.setName("m3");
	m4.setName("m4");
		
	EXPECT_TRUE(std::isnan(m1.getOut().getSignal().getValue()));
	EXPECT_EQ(m2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
	EXPECT_TRUE(std::isnan(m3.getOut().getSignal().getValue()[0]));
	EXPECT_TRUE(std::isnan(m3.getOut().getSignal().getValue()[0]));
	EXPECT_EQ(m4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
	EXPECT_EQ(m4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
	try {
		m1.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'm1'"));
	}
	try {
		m2.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'm2'"));
	}
	try {
		m3.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'm3'"));
	}
	try {
		m4.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'm4'"));
	}
}

TEST(controlMulTest, int) {
	Mul<int,int,int> m1;
	Constant<int> c1(10), c2(-325);
	m1.getIn1().connect(c1.getOut());
	m1.getIn2().connect(c2.getOut());
	c1.run(); c2.run(); m1.run();	
	EXPECT_EQ(m1.getOut().getSignal().getValue(), -3250);
}

TEST(controlMulTest, double) {
	Mul<> m1;
	Constant<> c1(1.3), c2(-0.5);
	m1.getIn1().connect(c1.getOut());
	m1.getIn2().connect(c2.getOut());
	c1.run(); c2.run(); m1.run();	
	EXPECT_EQ(m1.getOut().getSignal().getValue(), -0.65);
}

TEST(controlMulTest, m12) {
	Mul<Matrix<2,1,double>,Matrix<1,3,double>,Matrix<2,3,double>> m1;
	Constant<Matrix<2,1,double>> c1({-2,1});
	Constant<Matrix<1,3,double>> c2({2,3,-4});
	m1.getIn1().connect(c1.getOut());
	m1.getIn2().connect(c2.getOut());
	c1.run(); c2.run(); m1.run();
	Constant<Matrix<2,3,double>> c3({-4,2,-6,3,8,-4}); c3.run();
	EXPECT_EQ(m1.getOut().getSignal().getValue(), c3.getOut().getSignal().getValue());
}