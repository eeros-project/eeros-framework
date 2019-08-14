#include <eeros/control/Switch.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <Utils.hpp>
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
	s1.combine(s2);
	EXPECT_EQ(s1.getCurrentInput(), 1);
	EXPECT_EQ(s2.getCurrentInput(), 1);
	EXPECT_EQ(s3.getCurrentInput(), 1);
	s1.switchToInput(0);
	EXPECT_EQ(s1.getCurrentInput(), 0);
	EXPECT_EQ(s2.getCurrentInput(), 0);
	EXPECT_EQ(s3.getCurrentInput(), 1);
	s1.combine(s3);
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

// Test switching
TEST(controlSwitchTest, switching) {
	Switch<> s1(0);
	Constant<> c1(1), c2(2);
	s1.getIn(0).connect(c1.getOut());
	s1.getIn(1).connect(c2.getOut());
	c1.run();
	c2.run();
	s1.run();
	EXPECT_TRUE(Utils::compareApprox(s1.getOut().getSignal().getValue(), 1, 1e-10));
	s1.switchToInput(1);
	s1.run();
	EXPECT_TRUE(Utils::compareApprox(s1.getOut().getSignal().getValue(), 2, 1e-10));
	s1.setCondition(1.5, 0.1, 0);
	s1.run();
	EXPECT_TRUE(Utils::compareApprox(s1.getOut().getSignal().getValue(), 2, 1e-10));
	c2.setValue(1.55);
	c2.run();
	s1.run();
	EXPECT_EQ(s1.getCurrentInput(), 1);
	c2.setValue(2);
	c2.run();
	s1.run();
	EXPECT_EQ(s1.getCurrentInput(), 1);
	s1.arm();
	s1.run();
	EXPECT_EQ(s1.getCurrentInput(), 1);
	c2.setValue(1.55);
	c2.run();
	s1.run();
	EXPECT_EQ(s1.getCurrentInput(), 0);
}
