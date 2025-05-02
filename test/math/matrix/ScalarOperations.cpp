#include <eeros/control/DeMux.hpp>
#include <eeros/core/Runnable.hpp>
#include <eeros/math/Matrix.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::math;

TEST(mathMatrixScalarOps, add1) {
  Matrix<4, 1, double> m{1,-1,4,0};
  double scalar = 5;
  Matrix<4, 1, double> res = scalar + m;
  EXPECT_TRUE(Utils::compareApprox(6, res[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(4, res[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(9, res[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(5, res[3], 0.001));
}

TEST(mathMatrixScalarOps, add2) {
  Matrix<4, 1, double> m{1,-1,4,0};
  double scalar = 5;
  Matrix<4, 1, double> res = m + scalar;
  EXPECT_TRUE(Utils::compareApprox(6, res[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(4, res[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(9, res[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(5, res[3], 0.001));
}

TEST(mathMatrixScalarOps, sub1) {
  Matrix<4, 1, double> m{1,-1,4,0};
  double scalar = 5;
  Matrix<4, 1, double> res = scalar - m;
  EXPECT_TRUE(Utils::compareApprox(4, res[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(6, res[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(1, res[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(5, res[3], 0.001));
}

TEST(mathMatrixScalarOps, sub2) {
  Matrix<4, 1, double> m{1,-1,4,0};
  double scalar = 5;
  Matrix<4, 1, double> res = m - scalar;
  EXPECT_TRUE(Utils::compareApprox(-4, res[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(-6, res[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(-1, res[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(-5, res[3], 0.001));
}

TEST(mathMatrixScalarOps, mult1) {
  Matrix<4, 1, double> m{1,-1,4,0};
  double scalar = 5;
  Matrix<4, 1, double> res = scalar * m;
  EXPECT_TRUE(Utils::compareApprox(5, res[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(-5, res[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(20, res[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(0, res[3], 0.001));
}

TEST(mathMatrixScalarOps, mult2) {
  Matrix<4, 1, double> m{1,-1,4,0};
  double scalar = 5;
  Matrix<4, 1, double> res = m * scalar;
  EXPECT_TRUE(Utils::compareApprox(5, res[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(-5, res[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(20, res[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(0, res[3], 0.001));
}

TEST(mathMatrixScalarOps, div1) {
  Matrix<4, 1, double> m{1,-1,4,0};
  double scalar = 5;
  Matrix<4, 1, double> res = m / scalar;
  EXPECT_TRUE(Utils::compareApprox(0.2, res[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(-0.2, res[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(0.8, res[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(0, res[3], 0.001));
}

TEST(mathMatrixScalarOps, div2) {
  Matrix<4, 1, double> m{1,-1,4,100};
  double scalar = 5;
  Matrix<4, 1, double> res = scalar / m;
  EXPECT_TRUE(Utils::compareApprox(5, res[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(-5, res[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(1.25, res[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(0.05, res[3], 0.001));
}
