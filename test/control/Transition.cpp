#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/control/Transition.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


TEST(controlTransitionNameTest, naming) {
	Transition<> t1(1);
	EXPECT_EQ(t1.inBlock.getName(), std::string(""));
	t1.inBlock.setName("transition block up");
	EXPECT_EQ(t1.inBlock.getName(), std::string("transition block up"));
	Transition<> t2(1);
	EXPECT_EQ(t2.inBlock.getName(), std::string(""));
	t2.inBlock.setName("transition block down");
	EXPECT_EQ(t2.inBlock.getName(), std::string("transition block down"));
}

TEST(controlTransitionNanTest, nan) {
	Transition<> t1(1);
	t1.inBlock.setName("t1");
	EXPECT_TRUE(std::isnan(t1.outBlock.getOut().getSignal().getValue()));
	Transition<> t2(1);
	t2.inBlock.setName("t2");
	EXPECT_TRUE(std::isnan(t2.outBlock.getOut().getSignal().getValue()));
	try {
		t1.inBlock.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 't1'"));
	}
	try {
		t2.inBlock.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 't2'"));
	}
}

TEST(controlTransitionSimpleTest, simple) {
	Transition<> t1(5);
	Transition<> t2(1/5);
	Constant<> c1(0), c2(0);
	t1.inBlock.getIn().connect(c1.getOut());
	t2.inBlock.getIn().connect(t1.outBlock.getOut());
	t2.outBlock.getIn().connect(c2.getOut());
	c1.run();
	timestamp_t start = c1.getOut().getSignal().getTimestamp();
	t1.inBlock.run();
	usleep(1000);
	c2.run();	// fix the timestamp
	usleep(1000);
	c1.setValue(1.0);
	c1.run();
	timestamp_t end = c1.getOut().getSignal().getTimestamp();
	t1.inBlock.run();
	t1.outBlock.run();
	for (int i = 0; i < 5; i++) {
		EXPECT_EQ(t1.outBlock.getOut().getSignal().getTimestamp(), start + i * (end - start) / 5);
		EXPECT_TRUE(Utils::compareApprox(t1.outBlock.getOut().getSignal().getValue(), i * 0.2, 1e-10));
		t1.outBlock.run();
		t2.inBlock.run();
	}
	t2.outBlock.run();
	EXPECT_TRUE(Utils::compareApprox(t2.outBlock.getOut().getSignal().getValue(), 0.4, 1e-10));
}

TEST(controlTransitionSimpleTest, vector) {
	Transition<Vector2> t1(5);
	Transition<Vector2> t2(1/5);
	Constant<Vector2> c1({0,0}), c2({0,0});
	t1.inBlock.getIn().connect(c1.getOut());
	t2.inBlock.getIn().connect(t1.outBlock.getOut());
	t2.outBlock.getIn().connect(c2.getOut());
	c1.run();
	timestamp_t start = c1.getOut().getSignal().getTimestamp();
	t1.inBlock.run();
	usleep(1000);
	c2.run();	// fix the timestamp
	usleep(1000);
	c1.setValue({1.0, 2.0});
	c1.run();
	timestamp_t end = c1.getOut().getSignal().getTimestamp();
	t1.inBlock.run();
	t1.outBlock.run();
	EXPECT_EQ(t1.outBlock.getOut().getSignal().getValue()[0], 0);
	for (int i = 0; i < 5; i++) {
		EXPECT_EQ(t1.outBlock.getOut().getSignal().getTimestamp(), start + i * (end - start) / 5);
		EXPECT_TRUE(Utils::compareApprox(t1.outBlock.getOut().getSignal().getValue()[0], i * 0.2, 1e-10));
		EXPECT_TRUE(Utils::compareApprox(t1.outBlock.getOut().getSignal().getValue()[1], i * 0.4, 1e-10));
		t1.outBlock.run();
		t2.inBlock.run();
	}
	t2.outBlock.run();
	EXPECT_TRUE(Utils::compareApprox(t2.outBlock.getOut().getSignal().getValue()[0], 0.4, 1e-10));
	EXPECT_TRUE(Utils::compareApprox(t2.outBlock.getOut().getSignal().getValue()[1], 0.8, 1e-10));
}

TEST(controlTransitionSimpleTest, steady) {
	Transition<> t1(5, true);
	Transition<> t2(1/5, true);
	Constant<> c1(0), c2(0);
	t1.inBlock.getIn().connect(c1.getOut());
	t2.inBlock.getIn().connect(t1.outBlock.getOut());
	t2.outBlock.getIn().connect(c2.getOut());
	c1.run();
	timestamp_t start = c1.getOut().getSignal().getTimestamp();
	t1.inBlock.run();
	usleep(5000);
	c1.setValue(1.0);
	c1.run();
	timestamp_t end = c1.getOut().getSignal().getTimestamp();
	t1.inBlock.run();
	t1.outBlock.run();
	for (int i = 0; i < 5; i++) {
		EXPECT_EQ(t1.outBlock.getOut().getSignal().getTimestamp(), end);
		EXPECT_TRUE(Utils::compareApprox(t1.outBlock.getOut().getSignal().getValue(), 1.0, 1e-10));
		t1.outBlock.run();
		t2.inBlock.run();
	}
	t2.outBlock.run();
	EXPECT_EQ(t2.outBlock.getOut().getSignal().getTimestamp(), end);
	EXPECT_TRUE(Utils::compareApprox(t2.outBlock.getOut().getSignal().getValue(), 1.0, 1e-10));
}

