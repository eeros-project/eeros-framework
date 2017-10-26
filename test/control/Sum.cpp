#include <eeros/control/Sum.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test initial values for NaN
TEST(controlSumTest, initialValue) {
	Sum<> s1;
	Sum<2, int> s2;
	Sum<2, Matrix<2,1,double>> s3;
	Sum<2, Matrix<2,1,int>> s4;
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
	try {
		s1.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 's1'"));
	}
	try {
		s2.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 's2'"));
	}
	try {
		s3.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 's3'"));
	}
	try {
		s4.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 's4'"));
	}
}

TEST(controlSumTest, int) {
	Sum<2, int> s1;
	Constant<int> c1(10), c2(-325);
	s1.getIn(0).connect(c1.getOut());
	s1.getIn(1).connect(c2.getOut());
	c1.run(); c2.run(); s1.run();	
	EXPECT_EQ(s1.getOut().getSignal().getValue(), -315);
	s1.negateInput(0);
	s1.run();
	EXPECT_EQ(s1.getOut().getSignal().getValue(), -335);
	s1.negateInput(1);
	s1.run();
	EXPECT_EQ(s1.getOut().getSignal().getValue(), 315);
}