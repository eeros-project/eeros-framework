#include <gtest/gtest.h>

#include <eeros/core/Fault.hpp>
#include <eeros/math/Matrix.hpp>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::math;

template <uint8_t M, uint8_t N = 1>
struct uuT {
  Matrix<M, N> matrix;
  double det;
  uint32_t rank;
  double trace;
  bool orthogonal;
  bool invertible;
  bool symmetric;
  bool lowerTriangular;
  bool upperTriangular;
};

const double MAX_DEVIATION = 0.0001;  // relative
constexpr int NUMBER_OF_ROT_TESTING_DATA = 7;

TEST(mathMatrixCharacteristicsTest, swapRows) {
  Matrix<3, 3> testMatrix1, refResult1;
  testMatrix1 << 1, 1, 0, 0, 0, 2, 1, 0, 0;
  refResult1 << 0, 0, 2, 1, 1, 0, 1, 0, 0;
  testMatrix1.swapRows(0, 1);
  EXPECT_EQ(testMatrix1, refResult1);

  Matrix<4, 3> testMatrix2, refResult2;
  testMatrix2 << 1, 1, 0, 0, 0, 2, 1, 0, 0, 4, 5, 6;
  refResult2 << 4, 5, 6, 0, 0, 2, 1, 0, 0, 1, 1, 0;
  testMatrix2.swapRows(0, 3);
  EXPECT_EQ(testMatrix2, refResult2);
}


TEST(mathMatrixCharacteristicsTest, gaussSort) {
  Matrix<3, 3> testMatrix1, refResult1;
  testMatrix1 << 1, 1, 0, 0, 0, 2, 1, 0, 0;
  refResult1 << 1, 1, 0, 1, 0, 0, 0, 0, 2;
  testMatrix1.sortForGaussAlgorithm();
  EXPECT_EQ(testMatrix1, refResult1);

  Matrix<3, 3> testMatrix2, refResult2;
  testMatrix2 << 0, 1, 0, 0, 0, 2, 1, 0, 0;
  refResult2 << 1, 0, 0, 0, 1, 0, 0, 0, 2;
  testMatrix2.sortForGaussAlgorithm();
  EXPECT_EQ(testMatrix2, refResult2);

  Matrix<3, 3> testMatrix3, refResult3;
  testMatrix3 << 5, 1, 3, 2, 1, 2, 5.25, 3.125, 2.5;
  refResult3 << 5, 1, 3, 2, 1, 2, 5.25, 3.125, 2.5;
  testMatrix3.sortForGaussAlgorithm();
  EXPECT_EQ(testMatrix3, refResult3);

  Matrix<4, 3> testMatrix4, refResult4;
  testMatrix4 << 1, 1, 0, 0, 0, 2, 1, 0, 0, 4, 5, 6;
  refResult4 << 1, 1, 0, 1, 0, 0, 4, 5, 6, 0, 0, 2;
  testMatrix4.sortForGaussAlgorithm();
  EXPECT_EQ(testMatrix4, refResult4);

  Matrix<3, 4> testMatrix5, refResult5;
  testMatrix5 << 1, 1, 0, 0, 0, 0, 2, 2, 1, 0, 0, 0;
  refResult5 << 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 2, 2;
  testMatrix5.sortForGaussAlgorithm();
  EXPECT_EQ(testMatrix5, refResult5);

  Matrix<3, 4> testMatrix6, refResult6;
  testMatrix6 << 2, 4, 6, 8, 1, 2, 3, 4, 0, 1, 1, 1;
  refResult6 << 2, 4, 6, 8, 1, 2, 3, 4, 0, 1, 1, 1;
  testMatrix6.sortForGaussAlgorithm();
  EXPECT_EQ(testMatrix6, refResult6);
}

TEST(mathMatrixCharacteristicsTest, gaussRowElim) {
  Matrix<3, 3> testMatrix1, refResult1;
  testMatrix1 << 1, 1, 0, 0, 0, 2, 1, 0, 0;
  refResult1 << 1, 1, 0, 0, -1, 0, 0, 0, 2;
  testMatrix1.gaussRowElimination();
  EXPECT_EQ(testMatrix1, refResult1);

  Matrix<3, 3> testMatrix2, refResult2;
  testMatrix2 << 1, 3, 2, 2, 4, 4, 3, 5, 6;
  refResult2 << 1, 3, 2, 0, -2, 0, 0, 0, 0;
  testMatrix2.gaussRowElimination();
  EXPECT_EQ(testMatrix2, refResult2);

  Matrix<3, 4> testMatrix3, refResult3;
  testMatrix3 << 1, 2, 3, 4, -1, 0, 1, 0, 3, 5, 6, 9;
  refResult3 << 1, 2, 3, 4, 0, 2, 4, 4, 0, 0, -1, -1;
  testMatrix3.gaussRowElimination();
  EXPECT_EQ(testMatrix3, refResult3);

  Matrix<3, 4> testMatrix4, refResult4;
  testMatrix4 << 2, 4, 6, 8, 1, 2, 3, 5, 0, 1, 1, 1;
  refResult4 << 2, 4, 6, 8, 0, 0, 0, 1, 0, 1, 1, 1;
  testMatrix4.gaussRowElimination();
  EXPECT_EQ(testMatrix4, refResult4);
}

TEST(mathMatrixCharacteristicsTest, characteristics) {
  std::array<uuT<3, 3>, 2> characteristics33;
  characteristics33[0].matrix(0, 0) = 1;
  characteristics33[0].matrix(0, 1) = 0;
  characteristics33[0].matrix(0, 2) = 0;
  characteristics33[0].matrix(1, 0) = 0;
  characteristics33[0].matrix(1, 1) = 1;
  characteristics33[0].matrix(1, 2) = 0;
  characteristics33[0].matrix(2, 0) = 0;
  characteristics33[0].matrix(2, 1) = 0;
  characteristics33[0].matrix(2, 2) = 1;
  characteristics33[0].det = 1;
  characteristics33[0].rank = 3;
  characteristics33[0].trace = 3;
  characteristics33[0].orthogonal = true;
  characteristics33[0].invertible = true;
  characteristics33[0].symmetric = true;
  characteristics33[0].lowerTriangular = true;
  characteristics33[0].upperTriangular = true;

  characteristics33[1].matrix(0, 0) = 1;
  characteristics33[1].matrix(0, 1) = 3;
  characteristics33[1].matrix(0, 2) = 2;
  characteristics33[1].matrix(1, 0) = 2;
  characteristics33[1].matrix(1, 1) = 4;
  characteristics33[1].matrix(1, 2) = 4;
  characteristics33[1].matrix(2, 0) = 3;
  characteristics33[1].matrix(2, 1) = 5;
  characteristics33[1].matrix(2, 2) = 6;
  characteristics33[1].det = 0;
  characteristics33[1].rank = 2;
  characteristics33[1].trace = 11;
  characteristics33[1].orthogonal = false;
  characteristics33[1].invertible = false;
  characteristics33[1].symmetric = false;
  characteristics33[1].lowerTriangular = false;
  characteristics33[1].upperTriangular = false;

  std::array<uuT<2, 2>, 1> characteristics22;
  characteristics22[0].matrix(0, 0) = 2;
  characteristics22[0].matrix(0, 1) = 5;
  characteristics22[0].matrix(1, 0) = 4;
  characteristics22[0].matrix(1, 1) = 6;
  characteristics22[0].det = -8;
  characteristics22[0].rank = 2;
  characteristics22[0].trace = 8;
  characteristics22[0].orthogonal = false;
  characteristics22[0].invertible = true;
  characteristics22[0].symmetric = false;
  characteristics22[0].lowerTriangular = false;
  characteristics22[0].upperTriangular = false;

  std::array<uuT<4, 4>, 4> characteristics44;
  characteristics44[0].matrix(0, 0) = 1;
  characteristics44[0].matrix(0, 1) = 3;
  characteristics44[0].matrix(0, 2) = 2;
  characteristics44[0].matrix(0, 3) = 6;
  characteristics44[0].matrix(1, 0) = 2;
  characteristics44[0].matrix(1, 1) = 4;
  characteristics44[0].matrix(1, 2) = 4;
  characteristics44[0].matrix(1, 3) = 3;
  characteristics44[0].matrix(2, 0) = 3;
  characteristics44[0].matrix(2, 1) = 5;
  characteristics44[0].matrix(2, 2) = 6;
  characteristics44[0].matrix(2, 3) = 1;
  characteristics44[0].matrix(3, 0) = 7;
  characteristics44[0].matrix(3, 1) = 9;
  characteristics44[0].matrix(3, 2) = 1;
  characteristics44[0].matrix(3, 3) = 4;
  characteristics44[0].det = -26;
  characteristics44[0].rank = 4;
  characteristics44[0].trace = 15;
  characteristics44[0].orthogonal = false;
  characteristics44[0].invertible = true;
  characteristics44[0].symmetric = false;
  characteristics44[0].lowerTriangular = false;
  characteristics44[0].upperTriangular = false;

  characteristics44[1].matrix(0, 0) = 3;
  characteristics44[1].matrix(0, 1) = 7;
  characteristics44[1].matrix(0, 2) = 3;
  characteristics44[1].matrix(0, 3) = 0;
  characteristics44[1].matrix(1, 0) = 0;
  characteristics44[1].matrix(1, 1) = 2;
  characteristics44[1].matrix(1, 2) = -1;
  characteristics44[1].matrix(1, 3) = 1;
  characteristics44[1].matrix(2, 0) = 5;
  characteristics44[1].matrix(2, 1) = 4;
  characteristics44[1].matrix(2, 2) = 3;
  characteristics44[1].matrix(2, 3) = 2;
  characteristics44[1].matrix(3, 0) = 6;
  characteristics44[1].matrix(3, 1) = 6;
  characteristics44[1].matrix(3, 2) = 4;
  characteristics44[1].matrix(3, 3) = -1;
  characteristics44[1].det = 105;
  characteristics44[1].rank = 4;
  characteristics44[1].trace = 7;
  characteristics44[1].orthogonal = false;
  characteristics44[1].invertible = true;
  characteristics44[1].symmetric = false;
  characteristics44[1].lowerTriangular = false;
  characteristics44[1].upperTriangular = false;

  characteristics44[2].matrix(0, 0) = 3;
  characteristics44[2].matrix(0, 1) = 0;
  characteristics44[2].matrix(0, 2) = 0;
  characteristics44[2].matrix(0, 3) = 0;
  characteristics44[2].matrix(1, 0) = 1;
  characteristics44[2].matrix(1, 1) = 2;
  characteristics44[2].matrix(1, 2) = 0;
  characteristics44[2].matrix(1, 3) = 0;
  characteristics44[2].matrix(2, 0) = 2;
  characteristics44[2].matrix(2, 1) = 3;
  characteristics44[2].matrix(2, 2) = 3;
  characteristics44[2].matrix(2, 3) = 0;
  characteristics44[2].matrix(3, 0) = 3;
  characteristics44[2].matrix(3, 1) = 2;
  characteristics44[2].matrix(3, 2) = 4;
  characteristics44[2].matrix(3, 3) = -1;
  characteristics44[2].det = -18;
  characteristics44[2].rank = 4;
  characteristics44[2].trace = 7;
  characteristics44[2].orthogonal = false;
  characteristics44[2].invertible = true;
  characteristics44[2].symmetric = false;
  characteristics44[2].lowerTriangular = true;
  characteristics44[2].upperTriangular = false;

  characteristics44[3].matrix(0, 0) = 3;
  characteristics44[3].matrix(0, 1) = 4.20;
  characteristics44[3].matrix(0, 2) = 1.3;
  characteristics44[3].matrix(0, 3) = 0.340;
  characteristics44[3].matrix(1, 0) = 0.0;
  characteristics44[3].matrix(1, 1) = 2;
  characteristics44[3].matrix(1, 2) = 1.3;
  characteristics44[3].matrix(1, 3) = 2;
  characteristics44[3].matrix(2, 0) = 0.0;
  characteristics44[3].matrix(2, 1) = 0.0;
  characteristics44[3].matrix(2, 2) = 3.10;
  characteristics44[3].matrix(2, 3) = 1.90;
  characteristics44[3].matrix(3, 0) = 0.0;
  characteristics44[3].matrix(3, 1) = 0.0;
  characteristics44[3].matrix(3, 2) = 0.0;
  characteristics44[3].matrix(3, 3) = -0.10;
  characteristics44[3].det = -1.86;
  characteristics44[3].rank = 4;
  characteristics44[3].trace = 8;
  characteristics44[3].orthogonal = false;
  characteristics44[3].invertible = true;
  characteristics44[3].symmetric = false;
  characteristics44[3].lowerTriangular = false;
  characteristics44[3].upperTriangular = true;

  // rank
  for (uint32_t i = 0; i < characteristics22.size(); i++) {
    double res = characteristics22[i].matrix.rank();
    EXPECT_EQ(res, characteristics22[i].rank);
  }

  for (uint32_t i = 0; i < characteristics33.size(); i++) {
    double res = characteristics33[i].matrix.rank();
    EXPECT_EQ(res, characteristics33[i].rank);
  }

  for (uint32_t i = 0; i < characteristics44.size(); i++) {
    double res = characteristics44[i].matrix.rank();
    EXPECT_EQ(res, characteristics44[i].rank);
  }

  // determinant
  for (uint32_t i = 0; i < characteristics22.size(); i++) {
    double res = characteristics22[i].matrix.det();
    EXPECT_TRUE(Utils::compareApprox(res, characteristics22[i].det, MAX_DEVIATION));
  }

  for (uint32_t i = 0; i < characteristics33.size(); i++) {
    double res = characteristics33[i].matrix.det();
    EXPECT_TRUE(Utils::compareApprox(res, characteristics33[i].det, MAX_DEVIATION));
  }

  for (uint32_t i = 0; i < characteristics44.size(); i++) {
    double res = characteristics44[i].matrix.det();
    EXPECT_TRUE(Utils::compareApprox(res, characteristics44[i].det, MAX_DEVIATION));
  }

  // trace

  for (uint32_t i = 0; i < characteristics22.size(); i++) {
    double res = characteristics22[i].matrix.trace();
    EXPECT_TRUE(Utils::compareApprox(res, characteristics22[i].trace, MAX_DEVIATION));
  }

  for (uint32_t i = 0; i < characteristics33.size(); i++) {
    double res = characteristics33[i].matrix.trace();
    EXPECT_TRUE(Utils::compareApprox(res, characteristics33[i].trace, MAX_DEVIATION));
  }

  for (uint32_t i = 0; i < characteristics44.size(); i++) {
    double res = characteristics44[i].matrix.trace();
    EXPECT_TRUE(Utils::compareApprox(res, characteristics44[i].trace, MAX_DEVIATION));
  }

  // orthogonal
  for (uint32_t i = 0; i < characteristics22.size(); i++) {
    EXPECT_EQ(characteristics22[i].orthogonal, characteristics22[i].matrix.isOrthogonal());
  }

  for (uint32_t i = 0; i < characteristics33.size(); i++) {
    EXPECT_EQ(characteristics33[i].orthogonal, characteristics33[i].matrix.isOrthogonal());
  }

  for (uint32_t i = 0; i < characteristics44.size(); i++) {
    EXPECT_EQ(characteristics44[i].orthogonal, characteristics44[i].matrix.isOrthogonal());
  }

  // invertible
  for (uint32_t i = 0; i < characteristics22.size(); i++) {
    EXPECT_EQ(characteristics22[i].invertible, characteristics22[i].matrix.isInvertible());
  }

  for (uint32_t i = 0; i < characteristics33.size(); i++) {
    EXPECT_EQ(characteristics33[i].invertible, characteristics33[i].matrix.isInvertible());
  }

  for (uint32_t i = 0; i < characteristics44.size(); i++) {
    EXPECT_EQ(characteristics44[i].invertible, characteristics44[i].matrix.isInvertible());
  }

  // symmetric
  for (uint32_t i = 0; i < characteristics22.size(); i++) {
    EXPECT_EQ(characteristics22[i].symmetric, characteristics22[i].matrix.isSymmetric());
  }

  for (uint32_t i = 0; i < characteristics33.size(); i++) {
    EXPECT_EQ(characteristics33[i].symmetric, characteristics33[i].matrix.isSymmetric());
  }

  for (uint32_t i = 0; i < characteristics44.size(); i++) {
    EXPECT_EQ(characteristics44[i].symmetric, characteristics44[i].matrix.isSymmetric());
  }

  // is lower triagular
  for (uint32_t i = 0; i < characteristics22.size(); i++) {
    EXPECT_EQ(characteristics22[i].lowerTriangular, characteristics22[i].matrix.isLowerTriangular());
  }

  for (uint32_t i = 0; i < characteristics33.size(); i++) {
    EXPECT_EQ(characteristics33[i].lowerTriangular, characteristics33[i].matrix.isLowerTriangular());
  }

  for (uint32_t i = 0; i < characteristics44.size(); i++) {
    EXPECT_EQ(characteristics44[i].lowerTriangular, characteristics44[i].matrix.isLowerTriangular());
  }

  // is upper triagular
  for (uint32_t i = 0; i < characteristics22.size(); i++) {
    EXPECT_EQ(characteristics22[i].upperTriangular, characteristics22[i].matrix.isUpperTriangular());
  }

  for (uint32_t i = 0; i < characteristics33.size(); i++) {
    EXPECT_EQ(characteristics33[i].upperTriangular, characteristics33[i].matrix.isUpperTriangular());
  }

  for (uint32_t i = 0; i < characteristics44.size(); i++) {
    EXPECT_EQ(characteristics44[i].upperTriangular, characteristics44[i].matrix.isUpperTriangular());
  }
}
