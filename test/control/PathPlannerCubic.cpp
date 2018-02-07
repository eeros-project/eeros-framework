#include <eeros/control/PathPlannerCubic.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test name
TEST(controlPathPlannerCubicTest, name) {
	PathPlannerCubic planner(0.1);
	EXPECT_EQ(planner.getName(), std::string(""));
	planner.setName("path planner");
	EXPECT_EQ(planner.getName(), std::string("path planner"));
}

// Test initial values for NaN
TEST(controlPathPlannerCubicTest, nan) {
	PathPlannerCubic planner(0.1);
	EXPECT_TRUE(std::isnan(planner.getJerkOut().getSignal().getValue()));
	EXPECT_TRUE(std::isnan(planner.getAccOut().getSignal().getValue()));
	EXPECT_TRUE(std::isnan(planner.getVelOut().getSignal().getValue()));
	EXPECT_TRUE(std::isnan(planner.getPosOut().getSignal().getValue()));
}

// Test initial positions
TEST(controlPathPlannerCubicTest, init1) {
	PathPlannerCubic planner(0.1);
	planner.init("path1.txt");
	planner.setInitPos(200);
	planner.move(10, 200, 1000);
	planner.run();
	EXPECT_EQ(planner.getJerkOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getAccOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getVelOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getPosOut().getSignal().getValue(), 200);
	for (int i = 0; i < 101; i++) planner.run();
	EXPECT_TRUE(Utils::compareApprox(planner.getJerkOut().getSignal().getValue(), 0, 1e-10));
	EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue(), 0, 1e-10));
	EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue(), 0, 1e-10));
	EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue(), 1200, 1e-10));
}

// Test initial positions
TEST(controlPathPlannerCubicTest, init2) {
	PathPlannerCubic planner(0.08);
	planner.init("path1.txt");
	planner.setInitPos(200);
	planner.move(200);
	planner.run();
	EXPECT_EQ(planner.getJerkOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getAccOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getVelOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getPosOut().getSignal().getValue(), 200);
	planner.run();
	EXPECT_EQ(planner.getJerkOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getAccOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getVelOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getPosOut().getSignal().getValue(), 200);
	planner.run();
	EXPECT_EQ(planner.getJerkOut().getSignal().getValue(), 10000);
	EXPECT_EQ(planner.getAccOut().getSignal().getValue(), 0);
	EXPECT_EQ(planner.getVelOut().getSignal().getValue(), 0);
	EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue(), 200.0, 1e-3));
	planner.run();
	EXPECT_EQ(planner.getJerkOut().getSignal().getValue(), 10000);
	EXPECT_EQ(planner.getAccOut().getSignal().getValue(), 800);
	EXPECT_EQ(planner.getVelOut().getSignal().getValue(), 32);
	EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue(), 200.853, 1e-3));
	for (int i = 0; i < 100; i++) planner.run();
	EXPECT_TRUE(Utils::compareApprox(planner.getJerkOut().getSignal().getValue(), 0, 1e-10));
	EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue(), 0, 1e-10));
	EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue(), 0, 1e-10));
	EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue(), 1183.04, 1e-3));
}
