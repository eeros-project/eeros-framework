#include <eeros/control/PathPlannerConstAcc.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/math/Matrix.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test name
TEST(controlPathPlannerConstAcc, name) {
  PathPlannerConstAcc<Matrix<1,1,double>> planner(1, 1, 1, 0.1);
  EXPECT_EQ(planner.getName(), std::string(""));
  planner.setName("path planner");
  EXPECT_EQ(planner.getName(), std::string("path planner"));
}

// Test initial values for NaN
TEST(controlPathPlannerConstAcc, nan) {
  PathPlannerConstAcc<Matrix<2,1,double>> planner({1,1}, {1,1}, {1,1}, 0.1);
  EXPECT_TRUE(std::isnan(planner.getAccOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(planner.getAccOut().getSignal().getValue()[1]));
  EXPECT_TRUE(std::isnan(planner.getVelOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(planner.getVelOut().getSignal().getValue()[1]));
  EXPECT_TRUE(std::isnan(planner.getPosOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(planner.getPosOut().getSignal().getValue()[1]));
}

// Test initial and end positions
TEST(controlPathPlannerConstAcc, init1) {
  PathPlannerConstAcc<Matrix<2,1,double>> planner({1,1}, {1,1}, {1,1}, 0.1);
  Matrix<2,1,double> start{10, 5}, end{20, 15};
  planner.move(start, end);
  planner.run();
  EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue()[0], 1, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue()[1], 1, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue()[0], 0.1, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue()[1], 0.1, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue()[0], 10.005, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue()[1], 5.005, 1e-10));
  for (int i = 0; i < 150; i++) planner.run();
  EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue()[0], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue()[1], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue()[0], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue()[1], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue()[0], 20, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue()[1], 15, 1e-10));
}

// Test initial and end positions
TEST(controlPathPlannerConstAcc, init2) {
  PathPlannerConstAcc<Matrix<2,1,double>> planner({1,1}, {1,1}, {1,1}, 0.1);
  Matrix<2,1,double> start{10, 5}, end{10, 15};
  planner.move(start, end);
  planner.run();
  EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue()[0], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue()[1], 1, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue()[0], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue()[1], 0.1, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue()[0], 10, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue()[1], 5.005, 1e-10));
  for (int i = 0; i < 150; i++) planner.run();
//   EXPECT_EQ(planner.getAccOut().getSignal().getValue()[0], 200);
//   EXPECT_EQ(planner.getVelOut().getSignal().getValue()[0], 200);
//   EXPECT_EQ(planner.getPosOut().getSignal().getValue()[0], 200);
  EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue()[0], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getAccOut().getSignal().getValue()[1], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue()[0], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getVelOut().getSignal().getValue()[1], 0, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue()[0], 10, 1e-10));
  EXPECT_TRUE(Utils::compareApprox(planner.getPosOut().getSignal().getValue()[1], 15, 1e-10));
}
