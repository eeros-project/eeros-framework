#include <array>
#include <cmath>
#include <cstdlib>
#include <eeros/math/Matrix.hpp>
#include <iostream>

#include "../../Utils.hpp"

using namespace eeros::math;

template <uint8_t M, uint8_t N, typename T>
void print(Matrix<M, N, T> &A, int indent = 1) {
  for (int m = 0; m < M; m++) {
    for (int i = 0; i < indent; i++) std::cout << '\t';
    for (int n = 0; n < N; n++) {
      if (n > 0) std::cout << '\t';
      std::cout << A(m, n);
    }
    std::cout << std::endl;
  }
}

template <uint8_t M, uint8_t N = 1>
struct uuT {
  Matrix<M, N> matrix;
  double det;
  uint32_t rank;
  double trace;
  uint8_t orthogonaly;
  uint8_t invertible;
  uint8_t symetric;
  uint8_t lowerTriangular;
  uint8_t upperTriangular;
};

const double MAX_DEVIATION = 0.0001;  // relative
constexpr int NUMBER_OF_ROT_TESTING_DATA = 7;

int testSwapingRows() {
  int error = 0;

  Matrix<3, 3> testMatrix1, refResult1;
  testMatrix1 << 1, 1, 0, 0, 0, 2, 1, 0, 0;

  refResult1 << 0, 0, 2, 1, 1, 0, 1, 0, 0;

  Matrix<4, 3> testMatrix2, refRestult2;
  testMatrix2 << 1, 1, 0, 0, 0, 2, 1, 0, 0, 4, 5, 6;

  refRestult2 << 4, 5, 6, 0, 0, 2, 1, 0, 0, 1, 1, 0;

  testMatrix1.swapRows(0, 1);
  testMatrix2.swapRows(0, 3);

  if (testMatrix1 != refResult1) {
    std::cout << "    -> Failure in result of test matrix 1, rwos were not "
                 "correctly swapped!"
              << std::endl;
    error++;
  }

  if (testMatrix2 != refRestult2) {
    std::cout << "    -> Failure in result of test matrix 2, rwos were not "
                 "correctly swapped!"
              << std::endl;
    error++;
  }
  return error;
}

int testGaussSorting() {
  int error = 0;

  Matrix<3, 3> testMatrix1, refResult1;
  testMatrix1 << 1, 1, 0, 0, 0, 2, 1, 0, 0;

  refResult1 << 1, 1, 0, 1, 0, 0, 0, 0, 2;

  Matrix<3, 3> testMatrix2, refResult2;
  testMatrix2 << 0, 1, 0, 0, 0, 2, 1, 0, 0;

  refResult2 << 1, 0, 0, 0, 1, 0, 0, 0, 2;

  Matrix<3, 3> testMatrix3, refResult3;
  testMatrix3 << 5, 1, 3, 2, 1, 2, 5.25, 3.125, 2.5;

  refResult3 << 5, 1, 3, 2, 1, 2, 5.25, 3.125, 2.5;

  Matrix<4, 3> testMatrix4, refResult4;
  testMatrix4 << 1, 1, 0, 0, 0, 2, 1, 0, 0, 4, 5, 6;

  refResult4 << 1, 1, 0, 1, 0, 0, 4, 5, 6, 0, 0, 2;

  Matrix<3, 4> testMatrix5, refResult5;
  testMatrix5 << 1, 1, 0, 0, 0, 0, 2, 2, 1, 0, 0, 0;

  refResult5 << 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 2, 2;

  Matrix<3, 4> testMatrix6, refResult6;
  testMatrix6 << 2, 4, 6, 8, 1, 2, 3, 4, 0, 1, 1, 1;

  refResult6 << 2, 4, 6, 8, 1, 2, 3, 4, 0, 1, 1, 1;

  testMatrix1.sortForGaussAlgorithm();
  testMatrix2.sortForGaussAlgorithm();
  testMatrix3.sortForGaussAlgorithm();
  testMatrix4.sortForGaussAlgorithm();
  testMatrix5.sortForGaussAlgorithm();
  testMatrix6.sortForGaussAlgorithm();

  if (testMatrix1 != refResult1) {
    std::cout << "    -> Failure in result of test matrix 1, matrix not "
                 "correctly sorted!"
              << std::endl;
    error++;
  }

  if (testMatrix2 != refResult2) {
    std::cout << "    -> Failure in result of test matrix 2, matrix not "
                 "correctly sorted!"
              << std::endl;
    error++;
  }

  if (testMatrix3 != refResult3) {
    std::cout << "    -> Failure in result of test matrix 3, matrix not "
                 "correctly sorted!"
              << std::endl;
    error++;
  }
  if (testMatrix4 != refResult4) {
    std::cout << "    -> Failure in result of test matrix 4, matrix not "
                 "correctly sorted!"
              << std::endl;
    error++;
  }
  if (testMatrix5 != refResult5) {
    std::cout << "    -> Failure in result of test matrix 5, matrix not "
                 "correctly sorted!"
              << std::endl;
    error++;
  }

  if (testMatrix6 != refResult6) {
    std::cout << "    -> Failure in result of test matrix 6, matrix not "
                 "correctly sorted!"
              << std::endl;
    error++;
  }
  return error;
}

int testGaussRowElimination() {
  int error = 0;
  Matrix<3, 3> testMatrix1, refResult1;
  testMatrix1 << 1, 1, 0, 0, 0, 2, 1, 0, 0;

  refResult1 << 1, 1, 0, 0, -1, 0, 0, 0, 2;

  Matrix<3, 3> testMatrix2, refResult2;
  testMatrix2 << 1, 3, 2, 2, 4, 4, 3, 5, 6;

  refResult2 << 1, 3, 2, 0, -2, 0, 0, 0, 0;

  Matrix<3, 4> testMatrix3, refResult3;
  testMatrix3 << 1, 2, 3, 4, -1, 0, 1, 0, 3, 5, 6, 9;

  refResult3 << 1, 2, 3, 4, 0, 2, 4, 4, 0, 0, -1, -1;

  Matrix<3, 4> testMatrix4, refResult4;
  testMatrix4 << 2, 4, 6, 8, 1, 2, 3, 5, 0, 1, 1, 1;

  refResult4 << 2, 4, 6, 8, 0, 0, 0, 1, 0, 1, 1, 1;

  testMatrix1.gaussRowElimination();
  testMatrix2.gaussRowElimination();
  testMatrix3.gaussRowElimination();
  testMatrix4.gaussRowElimination();

  if (testMatrix1 != refResult1) {
    std::cout << "    -> Failure in result of test matrix 1!" << std::endl;
    error++;
  }

  if (testMatrix2 != refResult2) {
    std::cout << "    -> Failure in result of test matrix 2!" << std::endl;
    error++;
  }

  if (testMatrix3 != refResult3) {
    std::cout << "    -> Failure in result of test matrix 3!" << std::endl;
    error++;
  }

  if (testMatrix4 != refResult4) {
    std::cout << "    -> Failure in result of test matrix 4!" << std::endl;
    error++;
  }
  return error;
}

// int main(int argc, char *argv[]) {
//   int error = 0, errorSum = 0;
//   int testNo = 1;
//
//   std::cout << "Testing functions for calculating characteristics of a Matrix"
//             << std::endl;
//
//   std::cout << "[A] Testing helper functions" << std::endl;
//
//   // Swaping rows
//   std::cout << "    #" << testNo++ << ": Swaping rows: swapRows(a, b)"
//             << std::endl;
//   error = testSwapingRows();
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   // Gauss sorting matrix
//   std::cout << "    #" << testNo++ << ": Gauss sorting Matrix" << std::endl;
//   error = testGaussSorting();
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   // gauss row elimination
//   std::cout << "    #" << testNo++ << ": Gaus row elimination" << std::endl;
//   error = testGaussRowElimination();
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   std::cout
//       << "[B] Testing functions for calculating characteristics of a Matrix"
//       << std::endl;
//
//   std::array<uuT<3, 3>, 2> characteristics33;
//
//   characteristics33[0].matrix(0, 0) = 1;
//   characteristics33[0].matrix(0, 1) = 0;
//   characteristics33[0].matrix(0, 2) = 0;
//   characteristics33[0].matrix(1, 0) = 0;
//   characteristics33[0].matrix(1, 1) = 1;
//   characteristics33[0].matrix(1, 2) = 0;
//   characteristics33[0].matrix(2, 0) = 0;
//   characteristics33[0].matrix(2, 1) = 0;
//   characteristics33[0].matrix(2, 2) = 1;
//   characteristics33[0].det = 1;
//   characteristics33[0].rank = 3;
//   characteristics33[0].trace = 3;
//   characteristics33[0].orthogonaly = true;
//   characteristics33[0].invertible = true;
//   characteristics33[0].symetric = true;
//   characteristics33[0].lowerTriangular = true;
//   characteristics33[0].upperTriangular = true;
//
//   characteristics33[1].matrix(0, 0) = 1;
//   characteristics33[1].matrix(0, 1) = 3;
//   characteristics33[1].matrix(0, 2) = 2;
//   characteristics33[1].matrix(1, 0) = 2;
//   characteristics33[1].matrix(1, 1) = 4;
//   characteristics33[1].matrix(1, 2) = 4;
//   characteristics33[1].matrix(2, 0) = 3;
//   characteristics33[1].matrix(2, 1) = 5;
//   characteristics33[1].matrix(2, 2) = 6;
//   characteristics33[1].det = 0;
//   characteristics33[1].rank = 2;
//   characteristics33[1].trace = 11;
//   characteristics33[1].orthogonaly = false;
//   characteristics33[1].invertible = false;
//   characteristics33[1].symetric = false;
//   characteristics33[1].lowerTriangular = false;
//   characteristics33[1].upperTriangular = false;
//
//   std::array<uuT<2, 2>, 1> characteristics22;
//
//   characteristics22[0].matrix(0, 0) = 2;
//   characteristics22[0].matrix(0, 1) = 5;
//   characteristics22[0].matrix(1, 0) = 4;
//   characteristics22[0].matrix(1, 1) = 6;
//   characteristics22[0].det = -8;
//   characteristics22[0].rank = 2;
//   characteristics22[0].trace = 8;
//   characteristics22[0].orthogonaly = false;
//   characteristics22[0].invertible = true;
//   characteristics22[0].symetric = false;
//   characteristics22[0].lowerTriangular = false;
//   characteristics22[0].upperTriangular = false;
//
//   std::array<uuT<4, 4>, 4> characteristics44;
//
//   characteristics44[0].matrix(0, 0) = 1;
//   characteristics44[0].matrix(0, 1) = 3;
//   characteristics44[0].matrix(0, 2) = 2;
//   characteristics44[0].matrix(0, 3) = 6;
//   characteristics44[0].matrix(1, 0) = 2;
//   characteristics44[0].matrix(1, 1) = 4;
//   characteristics44[0].matrix(1, 2) = 4;
//   characteristics44[0].matrix(1, 3) = 3;
//   characteristics44[0].matrix(2, 0) = 3;
//   characteristics44[0].matrix(2, 1) = 5;
//   characteristics44[0].matrix(2, 2) = 6;
//   characteristics44[0].matrix(2, 3) = 1;
//   characteristics44[0].matrix(3, 0) = 7;
//   characteristics44[0].matrix(3, 1) = 9;
//   characteristics44[0].matrix(3, 2) = 1;
//   characteristics44[0].matrix(3, 3) = 4;
//   characteristics44[0].det = -26;
//   characteristics44[0].rank = 4;
//   characteristics44[0].trace = 15;
//   characteristics44[0].orthogonaly = false;
//   characteristics44[0].invertible = true;
//   characteristics44[0].symetric = false;
//   characteristics44[0].lowerTriangular = false;
//   characteristics44[0].upperTriangular = false;
//
//   characteristics44[1].matrix(0, 0) = 3;
//   characteristics44[1].matrix(0, 1) = 7;
//   characteristics44[1].matrix(0, 2) = 3;
//   characteristics44[1].matrix(0, 3) = 0;
//   characteristics44[1].matrix(1, 0) = 0;
//   characteristics44[1].matrix(1, 1) = 2;
//   characteristics44[1].matrix(1, 2) = -1;
//   characteristics44[1].matrix(1, 3) = 1;
//   characteristics44[1].matrix(2, 0) = 5;
//   characteristics44[1].matrix(2, 1) = 4;
//   characteristics44[1].matrix(2, 2) = 3;
//   characteristics44[1].matrix(2, 3) = 2;
//   characteristics44[1].matrix(3, 0) = 6;
//   characteristics44[1].matrix(3, 1) = 6;
//   characteristics44[1].matrix(3, 2) = 4;
//   characteristics44[1].matrix(3, 3) = -1;
//   characteristics44[1].det = 105;
//   characteristics44[1].rank = 4;
//   characteristics44[1].trace = 7;
//   characteristics44[1].orthogonaly = false;
//   characteristics44[1].invertible = true;
//   characteristics44[1].symetric = false;
//   characteristics44[1].lowerTriangular = false;
//   characteristics44[1].upperTriangular = false;
//
//   characteristics44[2].matrix(0, 0) = 3;
//   characteristics44[2].matrix(0, 1) = 0;
//   characteristics44[2].matrix(0, 2) = 0;
//   characteristics44[2].matrix(0, 3) = 0;
//   characteristics44[2].matrix(1, 0) = 1;
//   characteristics44[2].matrix(1, 1) = 2;
//   characteristics44[2].matrix(1, 2) = 0;
//   characteristics44[2].matrix(1, 3) = 0;
//   characteristics44[2].matrix(2, 0) = 2;
//   characteristics44[2].matrix(2, 1) = 3;
//   characteristics44[2].matrix(2, 2) = 3;
//   characteristics44[2].matrix(2, 3) = 0;
//   characteristics44[2].matrix(3, 0) = 3;
//   characteristics44[2].matrix(3, 1) = 2;
//   characteristics44[2].matrix(3, 2) = 4;
//   characteristics44[2].matrix(3, 3) = -1;
//   characteristics44[2].det = -18;
//   characteristics44[2].rank = 4;
//   characteristics44[2].trace = 7;
//   characteristics44[2].orthogonaly = false;
//   characteristics44[2].invertible = true;
//   characteristics44[2].symetric = false;
//   characteristics44[2].lowerTriangular = true;
//   characteristics44[2].upperTriangular = false;
//
//   characteristics44[3].matrix(0, 0) = 3;
//   characteristics44[3].matrix(0, 1) = 4.20;
//   characteristics44[3].matrix(0, 2) = 1.3;
//   characteristics44[3].matrix(0, 3) = 0.340;
//   characteristics44[3].matrix(1, 0) = 0.0;
//   characteristics44[3].matrix(1, 1) = 2;
//   characteristics44[3].matrix(1, 2) = 1.3;
//   characteristics44[3].matrix(1, 3) = 2;
//   characteristics44[3].matrix(2, 0) = 0.0;
//   characteristics44[3].matrix(2, 1) = 0.0;
//   characteristics44[3].matrix(2, 2) = 3.10;
//   characteristics44[3].matrix(2, 3) = 1.90;
//   characteristics44[3].matrix(3, 0) = 0.0;
//   characteristics44[3].matrix(3, 1) = 0.0;
//   characteristics44[3].matrix(3, 2) = 0.0;
//   characteristics44[3].matrix(3, 3) = -0.10;
//   characteristics44[3].det = -1.86;
//   characteristics44[3].rank = 4;
//   characteristics44[3].trace = 8;
//   characteristics44[3].orthogonaly = false;
//   characteristics44[3].invertible = true;
//   characteristics44[3].symetric = false;
//   characteristics44[3].lowerTriangular = false;
//   characteristics44[3].upperTriangular = true;
//
//   std::cout << "    #" << testNo++ << ": Rank of a matrix: rank()" << std::endl;
//   error = 0;
//
//   for (uint32_t i = 0; i < characteristics22.size(); i++) {
//     double res = characteristics22[i].matrix.rank();
//     if (res != characteristics22[i].rank) {
//       std::cout << "    -> Failure: Matrix 2x2 #" << i
//                 << ": rank not correct calculated " << res << ", but should be "
//                 << characteristics22[i].rank << ")." << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics33.size(); i++) {
//     double res = characteristics33[i].matrix.rank();
//     if (res != characteristics33[i].rank) {
//       std::cout << "    -> Failure: Matrix 3x3 #" << i
//                 << ": rank not correct calculated " << res << ", but should be "
//                 << characteristics33[i].rank << ")." << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics44.size(); i++) {
//     double res = characteristics44[i].matrix.rank();
//     if (res != characteristics44[i].rank) {
//       std::cout << "    -> Failure: Matrix 4x4 #" << i
//                 << ": rank not correct calculated " << res << ", but should be "
//                 << characteristics44[i].rank << ")." << std::endl;
//       error++;
//     }
//   }
//
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   std::cout << "    #" << testNo++ << ": Determinant of a matrix: det()"
//             << std::endl;
//   error = 0;
//
//   for (uint32_t i = 0; i < characteristics22.size(); i++) {
//     double res = characteristics22[i].matrix.det();
//     if (!Utils::compareApprox(characteristics22[i].det, res, MAX_DEVIATION)) {
//       std::cout << "    -> Failure: Matrix 2x2 #" << i
//                 << ": determinant not correct calculated " << res
//                 << ", but should be " << characteristics22[i].det << ")."
//                 << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics33.size(); i++) {
//     double res = characteristics33[i].matrix.det();
//     if (!Utils::compareApprox(characteristics33[i].det, res, MAX_DEVIATION)) {
//       std::cout << "    -> Failure: Matrix 3x3 #" << i
//                 << ": determinant not correct calculated " << res
//                 << ", but should be " << characteristics33[i].det << ")."
//                 << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics44.size(); i++) {
//     double res = characteristics44[i].matrix.det();
//     if (!Utils::compareApprox(characteristics44[i].det, res, MAX_DEVIATION)) {
//       std::cout << "    -> Failure: Matrix 4x4 #" << i
//                 << ": determinant not correct calculated " << res
//                 << ", but should be " << characteristics44[i].det << ")."
//                 << std::endl;
//       error++;
//     }
//   }
//
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   std::cout << "    #" << testNo++ << ": Trace of a matrix: trace()"
//             << std::endl;
//   error = 0;
//
//   for (uint32_t i = 0; i < characteristics22.size(); i++) {
//     double res = characteristics22[i].matrix.trace();
//     if (!Utils::compareApprox(characteristics22[i].trace, res, MAX_DEVIATION)) {
//       std::cout << "    -> Failure: Matrix 2x2 #" << i
//                 << ": trace not correct calculated " << res
//                 << ", but should be " << characteristics22[i].trace << ")."
//                 << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics33.size(); i++) {
//     double res = characteristics33[i].matrix.trace();
//     if (!Utils::compareApprox(characteristics33[i].trace, res, MAX_DEVIATION)) {
//       std::cout << "    -> Failure: Matrix 3x3 #" << i
//                 << ": trace not correct calculated " << res
//                 << ", but should be " << characteristics33[i].trace << ")."
//                 << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics44.size(); i++) {
//     double res = characteristics44[i].matrix.trace();
//     if (!Utils::compareApprox(characteristics44[i].trace, res, MAX_DEVIATION)) {
//       std::cout << "    -> Failure: Matrix 4x4 #" << i
//                 << ": trace not correct calculated " << res
//                 << ", but should be " << characteristics44[i].trace << ")."
//                 << std::endl;
//       error++;
//     }
//   }
//
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   std::cout << "    #" << testNo++
//             << ": Check if matrix is orhogonal: isOrthogonal()" << std::endl;
//   error = 0;
//
//   for (uint32_t i = 0; i < characteristics22.size(); i++) {
//     if (characteristics22[i].matrix.isOrthogonal() !=
//         characteristics22[i].orthogonaly) {
//       std::cout << "    -> Failure: Matrix 2x2 #" << i
//                 << ": orthogonality not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics33.size(); i++) {
//     if (characteristics33[i].matrix.isOrthogonal() !=
//         characteristics33[i].orthogonaly) {
//       std::cout << "    -> Failure: Matrix 3x3 #" << i
//                 << ": orthogonality not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics44.size(); i++) {
//     if (characteristics44[i].matrix.isOrthogonal() !=
//         characteristics44[i].orthogonaly) {
//       std::cout << "    -> Failure: Matrix 4x4 #" << i
//                 << ": orthogonality not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   std::cout << "    #" << testNo++
//             << ": Check if matrix is invertible: isInvertible()" << std::endl;
//   error = 0;
//
//   for (uint32_t i = 0; i < characteristics22.size(); i++) {
//     if (characteristics22[i].matrix.isInvertible() !=
//         characteristics22[i].invertible) {
//       std::cout << "    -> Failure: Matrix 2x2 #" << i
//                 << ": invertibility not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics33.size(); i++) {
//     if (characteristics33[i].matrix.isInvertible() !=
//         characteristics33[i].invertible) {
//       std::cout << "    -> Failure: Matrix 3x3 #" << i
//                 << ": invertibility not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics44.size(); i++) {
//     if (characteristics44[i].matrix.isInvertible() !=
//         characteristics44[i].invertible) {
//       std::cout << "    -> Failure: Matrix 4x4 #" << i
//                 << ": invertibility not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   std::cout << "    #" << testNo++
//             << ": Check if matrix is symetric: isSymmetric()" << std::endl;
//   error = 0;
//
//   for (uint32_t i = 0; i < characteristics22.size(); i++) {
//     if (characteristics22[i].matrix.isSymmetric() !=
//         characteristics22[i].symetric) {
//       std::cout << "    -> Failure: Matrix 2x2 #" << i
//                 << ": symmetry not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics33.size(); i++) {
//     if (characteristics33[i].matrix.isSymmetric() !=
//         characteristics33[i].symetric) {
//       std::cout << "    -> Failure: Matrix 3x3 #" << i
//                 << ": symmetry not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics44.size(); i++) {
//     if (characteristics44[i].matrix.isSymmetric() !=
//         characteristics44[i].symetric) {
//       std::cout << "    -> Failure: Matrix 4x4 #" << i
//                 << ": symmetry not correctly detected" << std::endl;
//       error++;
//     }
//   }
//
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   std::cout << "    #" << testNo++
//             << ": Check if matrix is lower triagular: isLowerTriangular()"
//             << std::endl;
//   error = 0;
//
//   for (uint32_t i = 0; i < characteristics22.size(); i++) {
//     if (characteristics22[i].matrix.isLowerTriangular() !=
//         characteristics22[i].lowerTriangular) {
//       std::cout << "    -> Failure: Matrix 2x2 #" << i
//                 << ": detection if matrix is lower triangular failed"
//                 << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics33.size(); i++) {
//     if (characteristics33[i].matrix.isLowerTriangular() !=
//         characteristics33[i].lowerTriangular) {
//       std::cout << "    -> Failure: Matrix 3x3 #" << i
//                 << ": detection if matrix is lower triangular failed"
//                 << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics44.size(); i++) {
//     if (characteristics44[i].matrix.isLowerTriangular() !=
//         characteristics44[i].lowerTriangular) {
//       std::cout << "    -> Failure: Matrix 4x4 #" << i
//                 << ": detection if matrix is lower triangular failed"
//                 << std::endl;
//       error++;
//     }
//   }
//
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   std::cout << "    #" << testNo++
//             << ": Check if matrix is upper triagular: isUpperTriangular()"
//             << std::endl;
//
//   for (uint32_t i = 0; i < characteristics22.size(); i++) {
//     if (characteristics22[i].matrix.isUpperTriangular() !=
//         characteristics22[i].upperTriangular) {
//       std::cout << "    -> Failure: Matrix 2x2 #" << i
//                 << ": detection if matrix is upper triangular failed"
//                 << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics33.size(); i++) {
//     if (characteristics33[i].matrix.isUpperTriangular() !=
//         characteristics33[i].upperTriangular) {
//       std::cout << "    -> Failure: Matrix 3x3 #" << i
//                 << ": detection if matrix is upper triangular failed"
//                 << std::endl;
//       error++;
//     }
//   }
//
//   for (uint32_t i = 0; i < characteristics44.size(); i++) {
//     if (characteristics44[i].matrix.isUpperTriangular() !=
//         characteristics44[i].upperTriangular) {
//       std::cout << "    -> Failure: Matrix 4x4 #" << i
//                 << ": detection if matrix is upper triangular failed"
//                 << std::endl;
//       error++;
//     }
//   }
//
//   errorSum += error;
//   std::cout << "    -> Test finished with " << error << " error(s)"
//             << std::endl;
//
//   if (errorSum == 0) {
//     std::cout << "Matrix characteristics test succeeded" << std::endl;
//   } else {
//     std::cout << "Matrix characteristics test failed with " << errorSum
//               << " error(s)" << std::endl;
//   }
//
//   return errorSum;
// }
