#include <eeros/control/I.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

TEST(controlITest, naming) {
	I<> i1;
	EXPECT_EQ(i1.getName(), std::string(""));
	i1.setName("integrator 1");
	EXPECT_EQ(i1.getName(), std::string("integrator 1"));
}

// Test initial values for NaN
TEST(controlITest, initialValue) {
	I<> i1;
	I<int> i2;
	I<Matrix<2,1,double>> i3;
	I<Matrix<2,1,int>> i4;
	i1.setName("i1");
	i2.setName("i2");
	i3.setName("i3");
	i4.setName("i4");
		
	EXPECT_TRUE(std::isnan(i1.getOut().getSignal().getValue()));
	EXPECT_EQ(i2.getOut().getSignal().getValue(), std::numeric_limits<int32_t>::min());
	EXPECT_TRUE(std::isnan(i3.getOut().getSignal().getValue()[0]));
	EXPECT_TRUE(std::isnan(i3.getOut().getSignal().getValue()[1]));
	EXPECT_EQ(i4.getOut().getSignal().getValue()[0], std::numeric_limits<int32_t>::min());
	EXPECT_EQ(i4.getOut().getSignal().getValue()[1], std::numeric_limits<int32_t>::min());
	try {
		i1.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'i1'"));
	}
	try {
		i2.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'i2'"));
	}
	try {
		i3.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'i3'"));
	}
	try {
		i4.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'i4'"));
	}
}

// test function
TEST(controlITest, running) {
	Constant<> c1(1.0);
	I<> i1;
	I<Matrix<2,1,double>> i2;
	i1.setName("i1");
	i2.setName("i2");
	i1.getIn().connect(c1.getOut());
	c1.run();
	i1.run();
	EXPECT_TRUE(std::isnan(i1.getOut().getSignal().getValue()));
	i1.setInitCondition(2.5);
	i1.run();
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), 2.5, 1e-10));
	i1.run();
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), 2.5, 1e-10));
	usleep(1000);
	c1.run();
	i1.run();
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), 2.5, 1e-10));
	i1.enable();
	i1.run();
	EXPECT_EQ(i1.getOut().getSignal().getValue(), 2.5);
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), 2.5, 1e-10));
	usleep(1000);
	c1.run();
	i1.run();
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), 2.501, 1e-3));
	usleep(1000);
	c1.run();
	i1.run();
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), 2.502, 1e-3));
	i1.disable();
	usleep(1000);
	c1.run();
	i1.run();
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), 2.502, 1e-3));
}


