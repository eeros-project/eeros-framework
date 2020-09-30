#include <eeros/control/PathPlannerConstJerk.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/math/Matrix.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test name
TEST(controlPathPlannerConstJerk, name) {
//   PathPlannerConstJerk<Matrix<2,1,double>> planner({1,1}, {1,1}, {1,1}, 0.1);
//   EXPECT_EQ(planner.getName(), std::string(""));
//   planner.setName("path planner");
//   EXPECT_EQ(planner.getName(), std::string("path planner"));
}

// Test initial values for NaN
TEST(controlPathPlannerConstJerk, nan) {
//   PathPlannerConstJerk<Matrix<2,1,double>> planner({1,1}, {1,1}, {1,1}, 0.1);
//   EXPECT_TRUE(std::isnan(planner.getJerkOut().getSignal().getValue()[0]));
//   EXPECT_TRUE(std::isnan(planner.getJerkOut().getSignal().getValue()[1]));
//   EXPECT_TRUE(std::isnan(planner.getAccOut().getSignal().getValue()[0]));
//   EXPECT_TRUE(std::isnan(planner.getAccOut().getSignal().getValue()[1]));
//   EXPECT_TRUE(std::isnan(planner.getVelOut().getSignal().getValue()[0]));
//   EXPECT_TRUE(std::isnan(planner.getVelOut().getSignal().getValue()[1]));
//   EXPECT_TRUE(std::isnan(planner.getPosOut().getSignal().getValue()[0]));
//   EXPECT_TRUE(std::isnan(planner.getPosOut().getSignal().getValue()[1]));
}

