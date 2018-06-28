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

// test limit
TEST(controlITest, limit) {
	Constant<> c1(100.0);
	I<> i1;
	i1.getIn().connect(c1.getOut());
	i1.enable();
	i1.setInitCondition(-0.2);
 	i1.setLimit(5.6, -3.2);
	for (int i = 0; i < 100; i++) {c1.run(); i1.run(); usleep(1000);}
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), 5.6, 0.15));
	c1.setValue(-100.0);
	for (int i = 0; i < 100; i++) {c1.run(); i1.run(); usleep(1000);}
	EXPECT_TRUE(Utils::compareApprox(i1.getOut().getSignal().getValue(), -3.2, 0.15));

	Constant<Matrix<2,1,double>> c2(100.0);
	I<Matrix<2,1,double>> i2;
	i2.getIn().connect(c2.getOut());
	i2.enable();
	i2.setInitCondition({-0.5, 0.5});
	i2.setLimit({5.6, 5.3}, -3.2);
	for (int i = 0; i < 100; i++) {c2.run(); i2.run(); usleep(1000);}
	EXPECT_TRUE(Utils::compareApprox(i2.getOut().getSignal().getValue()[0], 4.3, 0.1));
	EXPECT_TRUE(Utils::compareApprox(i2.getOut().getSignal().getValue()[1], 5.3, 0.1));
	c2.setValue(-100.0);
	i2.setInitCondition({-0.5, 0.5});
	for (int i = 0; i < 100; i++) {c2.run(); i2.run(); usleep(1000);}
	EXPECT_TRUE(Utils::compareApprox(i2.getOut().getSignal().getValue()[0], -3.2, 0.1));
	EXPECT_TRUE(Utils::compareApprox(i2.getOut().getSignal().getValue()[1], -2.2, 0.1));
}
