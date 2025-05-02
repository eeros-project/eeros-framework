#include <eeros/math/Matrix.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::math;

TEST(mathMatrixOps, det) {
  Matrix<2, 2, double> m1{1,2,3,4};
  EXPECT_TRUE(Utils::compareApprox(-2, m1.det(), 0.001));
  Matrix<3, 3, double> m2{1,2,3,2,0,5,0,1,-1};
  EXPECT_TRUE(Utils::compareApprox(5, m2.det(), 0.001));
}

TEST(mathMatrixOps, inv) {
  Matrix<2, 2, double> m1{1,2,3,4};
  m1 = m1.inverse();
  EXPECT_TRUE(Utils::compareApprox(-2, m1(0,0), 0.001));
  EXPECT_TRUE(Utils::compareApprox(1.5, m1(0,1), 0.001));
  EXPECT_TRUE(Utils::compareApprox(1, m1(1,0), 0.001));
  EXPECT_TRUE(Utils::compareApprox(-0.5, m1(1,1), 0.001));

  Matrix<3, 3, double> m2{1,2,3,2,0,5,0,1,-1};
  m2 = m2.inverse();
  EXPECT_TRUE(Utils::compareApprox(-1, m2(0,0), 0.001));
  EXPECT_TRUE(Utils::compareApprox(0.4, m2(0,1), 0.001));
  EXPECT_TRUE(Utils::compareApprox(0.4, m2(0,2), 0.001));
  EXPECT_TRUE(Utils::compareApprox(1, m2(1,0), 0.001));
  EXPECT_TRUE(Utils::compareApprox(-0.2, m2(1,1), 0.001));
  EXPECT_TRUE(Utils::compareApprox(-0.2, m2(1,2), 0.001));
  EXPECT_TRUE(Utils::compareApprox(2, m2(2,0), 0.001));
  EXPECT_TRUE(Utils::compareApprox(0.2, m2(2,1), 0.001));
  EXPECT_TRUE(Utils::compareApprox(-0.8, m2(2,2), 0.001));
}
