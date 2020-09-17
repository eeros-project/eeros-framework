#include <eeros/control/PathPlannerTrapezoid.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test name
TEST(controlPathPlannerTrapezoidTest, name) {
	PathPlannerTrapezoid<> planner(0.1, 0, 0, 0);
	EXPECT_EQ(planner.getName(), std::string(""));
	planner.setName("path planner");
	EXPECT_EQ(planner.getName(), std::string("path planner"));
}

// Test initial values for NaN
TEST(controlPathPlannerTrapezoidTest, nan) {
	PathPlannerTrapezoid<> planner(0.1, 0, 0, 0);
	EXPECT_TRUE(std::isnan(planner.getJerkOut().getSignal().getValue()));
	EXPECT_TRUE(std::isnan(planner.getAccOut().getSignal().getValue()));
	EXPECT_TRUE(std::isnan(planner.getVelOut().getSignal().getValue()));
	EXPECT_TRUE(std::isnan(planner.getPosOut().getSignal().getValue()));
}

