#include <gtest/gtest.h>

#include <eeros/core/Fault.hpp>
#include <eeros/math/Matrix.hpp>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::math;

const double MAX_DEVIATION = 0.0001;

// Copy constructor
TEST(mathMatrixInitializationTest, init1) {
  Matrix<4, 2, int> iM4x2;

  for (unsigned int m = 0; m < 4; m++) {
    for (unsigned int n = 0; n < 2; n++) {
      iM4x2(m, n) = m + n;
    }
  }
  Matrix<4, 2, int> iM4x2copy(iM4x2);
  for (unsigned int m = 0; m < 4; m++) {
    for (unsigned int n = 0; n < 2; n++) {
      EXPECT_EQ(iM4x2copy(m, n), iM4x2(m, n));
    }
  }
}

// Single value constructor
TEST(mathMatrixInitializationTest, init2) {
  Matrix<8, 5, int> iM8x5(27);
  for (unsigned int m = 0; m < 8; m++) {
    for (unsigned int n = 0; n < 5; n++) {
       EXPECT_EQ(iM8x5(m, n), 27);
    }
  }
}

// Multiple value constructor
TEST(mathMatrixInitializationTest, init3) {
  Matrix<3, 1, int> iM3x1(1, 2, 3);
  for (unsigned int m = 0; m < 3; m++) {
    EXPECT_EQ(iM3x1(m, 0), m + 1);
  }
}

// Multiple value constructor
TEST(mathMatrixInitializationTest, init4) {
  Matrix<6, 3, int> iM6x3(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18);
  int i = 1;
  for(unsigned int n = 0; n < 3; n++) {
    for(unsigned int m = 0; m < 6; m++) {
      EXPECT_EQ(iM6x3(m, n), i++);
    }
  }
}

// Multiple value constructor
TEST(mathMatrixInitializationTest, init5) {
  constexpr double c0 = -0.1;
  constexpr double c1 = 3.1415;
  constexpr double c2 = 10;
  Matrix<3, 1, double> dM3x1(c0, c1, c2);
  EXPECT_EQ(dM3x1(0, 0), c0);
  EXPECT_EQ(dM3x1(1, 0), c1);
  EXPECT_EQ(dM3x1(2, 0), c2);
}

// Zero matrix
TEST(mathMatrixInitializationTest, zero) {
  Matrix<4, 2, int> iM4x2;
  iM4x2.zero();
  for(unsigned int n = 0; n < 2; n++) {
    for(unsigned int m = 0; m < 4; m++) {
      EXPECT_EQ(iM4x2(m, n), 0);
    }
  }
  Matrix<4, 2, double> dM4x2;
  dM4x2.zero();
  for(unsigned int n = 0; n < 2; n++) {
    for(unsigned int m = 0; m < 4; m++) {
      EXPECT_EQ(dM4x2(m, n), 0);
    }
  }
  Matrix<1,1> m1;
  m1.zero();
  EXPECT_EQ(m1(0, 0), 0);
}

// Eye matrix
TEST(mathMatrixInitializationTest, eye) {
  Matrix<5, 5, int> iM5x5;
  iM5x5.eye();
  for(unsigned int n = 0; n < 5; n++) {
    for(unsigned int m = 0; m < 5; m++) {
      if (n == m) EXPECT_EQ(iM5x5(m, n), 1);
      else EXPECT_EQ(iM5x5(m, n), 0);
    }
  }
  Matrix<4, 2, double> dM5x5;
  dM5x5.eye();
  for(unsigned int n = 0; n < 5; n++) {
    for(unsigned int m = 0; m < 5; m++) {
      if (n == m) EXPECT_EQ(iM5x5(m, n), 1);
      else EXPECT_EQ(iM5x5(m, n), 0);
    }
  }
  Matrix<1,1> m1;
  m1.eye();
  EXPECT_EQ(m1(0, 0), 1);
}

// Diagonal matrix
TEST(mathMatrixInitializationTest, diag) {
  int diagV = 9;
  Matrix<5, 5, int> diagM = Matrix<5, 5, int>::createDiag(diagV);
  for(unsigned int m = 0; m < 5; m++) {
    for(unsigned int n = 0; n < 5; n++) {
      if(m == n) EXPECT_EQ(diagM(m, n), diagV);
      else EXPECT_EQ(diagM(m, n), 0);
    }
  }
}

// Skew symmetric matrix
TEST(mathMatrixInitializationTest, skew) {
  Matrix<3, 1, int> ssMatA, ssMatB;
  ssMatA(0) = 1;  ssMatA(1) = 2; ssMatA(2) = 3;
  ssMatB(0) = -5; ssMatB(1) = -3; ssMatB(2) = 0;
  Matrix<3, 3, int> ssRefResA, ssRefResB, ssRes;
  ssRefResA(0, 0) =  0; ssRefResA(0, 1) = -3; ssRefResA(0, 2) =  2;
  ssRefResA(1, 0) =  3; ssRefResA(1, 1) =  0; ssRefResA(1, 2) = -1;
  ssRefResA(2, 0) = -2; ssRefResA(2, 1) =  1; ssRefResA(2, 2) =  0;
  ssRefResB(0, 0) =  0; ssRefResB(0, 1) =  0; ssRefResB(0, 2) = -3;
  ssRefResB(1, 0) =  0; ssRefResB(1, 1) =  0; ssRefResB(1, 2) =  5;
  ssRefResB(2, 0) =  3; ssRefResB(2, 1) = -5; ssRefResB(2, 2) =  0;
  ssRes = Matrix<3, 3, int>::createSkewSymmetric(ssMatA);
  for(unsigned int m = 0; m < 3; m++) {
    for(unsigned int n = 0; n < 3; n++) {
      EXPECT_EQ(ssRes(m, n), ssRefResA(m, n));
    }
  }
  ssRes = Matrix<3, 3, int>::createSkewSymmetric(ssMatB);
  for(unsigned int m = 0; m < 3; m++) {
    for(unsigned int n = 0; n < 3; n++) {
      EXPECT_EQ(ssRes(m, n), ssRefResB(m, n));
    }
  }
}

// << operator
TEST(mathMatrixInitializationTest, inputOp1) {
  Matrix<3, 3, int> sMat;
  sMat << 1, 4, 7, 2, 5, 8, 3, 6, 9;
  int j = 1;
  for(unsigned int n = 0; n < 3; n++) {
    for(unsigned int m = 0; m < 3; m++) {
      EXPECT_EQ(sMat(m,n), j++);
    }
  }
}

// << operator
TEST(mathMatrixInitializationTest, inputOp2) {
  Matrix<4, 1, double> tMat;
  tMat << 0.1, 0.2, 0.3, 0.4;
  double l = 0.1;
  int j = 1;
  for(unsigned int m = 0; m < 4; m++) {
    EXPECT_TRUE(Utils::compareApprox(tMat(m, 0), l, MAX_DEVIATION));
    l += 0.1;
  }
}

// << operator
TEST(mathMatrixInitializationTest, inputOp3) {
  Matrix<3, 3, int> aMat1;
  for(int k = -4; k < 5; k++) {
    aMat1 = k;
    for(unsigned int n = 0; n < 3; n++) {
      for(unsigned int m = 0; m < 3; m++) {
        EXPECT_EQ(aMat1(m,n), k);
      }
    }
  }
}

// = operator
TEST(mathMatrixInitializationTest, initInt) {
  Matrix<3, 3, double> aMat2;
  for(int k = -4; k < 5; k++) {
    aMat2 = k;
    for(unsigned int n = 0; n < 3; n++) {
      for(unsigned int m = 0; m < 3; m++) {
        EXPECT_EQ(aMat2(m,n), k);
      }
    }
  }
}
