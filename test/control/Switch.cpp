#include <eeros/control/Switch.hpp>
#include <eeros/math/Matrix.hpp>

#include <gtest/gtest.h>


using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


TEST(controlSwitchTest, templateInstantiations) {
	Switch<> s1(0);
	Switch<3> s2{0};
	Switch<3,int> s3{0};
	Switch<2,double> s4{0};
	Switch<4,Matrix<2,2>> s5{1};
	ASSERT_TRUE(true);
}

// Test initial values for NaN
TEST(controlSwitchTest, initialValue) {
	Switch<> s1(0);
	s1.setName("s1");
	EXPECT_TRUE(std::isnan(s1.getOut().getSignal().getValue()));
	try {
		s1.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 's1'"));
	}
	EXPECT_TRUE(std::isnan(s1.getOut().getSignal().getValue()));
}

// Test combined switching
TEST(controlSwitchTest, combined) {
	Switch<> s1(0), s2(0), s3(1);
	EXPECT_EQ(s1.getCurrentInput(), 0);
	EXPECT_EQ(s2.getCurrentInput(), 0);
	EXPECT_EQ(s3.getCurrentInput(), 1);
	s1.switchToInput(1);
	EXPECT_EQ(s1.getCurrentInput(), 1);
	EXPECT_EQ(s2.getCurrentInput(), 0);
	EXPECT_EQ(s3.getCurrentInput(), 1);
	s1.connect(s2);
	EXPECT_EQ(s1.getCurrentInput(), 1);
	EXPECT_EQ(s2.getCurrentInput(), 1);
	EXPECT_EQ(s3.getCurrentInput(), 1);
	s1.switchToInput(0);
	EXPECT_EQ(s1.getCurrentInput(), 0);
	EXPECT_EQ(s2.getCurrentInput(), 0);
	EXPECT_EQ(s3.getCurrentInput(), 1);
	s1.connect(s3);
	EXPECT_EQ(s1.getCurrentInput(), 0);
	EXPECT_EQ(s2.getCurrentInput(), 0);
	EXPECT_EQ(s3.getCurrentInput(), 0);
	s1.switchToInput(1);
	EXPECT_EQ(s1.getCurrentInput(), 1);
	EXPECT_EQ(s2.getCurrentInput(), 1);
	EXPECT_EQ(s3.getCurrentInput(), 1);
	s2.switchToInput(0);
	EXPECT_EQ(s1.getCurrentInput(), 1);
	EXPECT_EQ(s2.getCurrentInput(), 0);
	EXPECT_EQ(s3.getCurrentInput(), 1);
	s1.switchToInput(0);
	EXPECT_EQ(s1.getCurrentInput(), 0);
	EXPECT_EQ(s2.getCurrentInput(), 0);
	EXPECT_EQ(s3.getCurrentInput(), 0);
}
