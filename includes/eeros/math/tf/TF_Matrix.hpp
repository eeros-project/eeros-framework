#pragma once

#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
#include <string>
#include <vector>

namespace eeros {
namespace math {
namespace tf {

using namespace eeros::math;

/**
 * Transformation matrix
 */
class TF_Matrix : public Matrix<4, 4, double> {
 public:
   /**
    * Creates a new transformation matrix
    * Default values are the identity matrix
    *
    * @param m - initial values
    */
   TF_Matrix(Matrix<4,4,double> m = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0})
   : Matrix<4,4,double>(m) { };

  /**
   * Sets the translation.
   *
   * @param trans - translation
   */
  void setTrans(Vector3 trans);

  /**
   * Gets the translation.
   *
   * @return translation
   */
  Matrix<3, 1, double> getTrans();

  /**
   * Sets the rotation.
   *
   * @param rot - rotation
   */
  void setRot(Matrix<3, 3, double> rot);

  /**
   * Gets the rotation.
   *
   * @return rotation
   */
  Matrix<3, 3, double> getRot();

  /**
   * Sets the matrix.
   *
   * @param m - transformation matrix
   */
  void setMatrix(Matrix<4, 4, double> m);

  /**
   * Gets the transformation matrix.
   *
   * @return transformation matrix
   */
  const Matrix<4, 4, double>& getMatrix() const;

  /**
   * Gets roll, pitch, yaw from tranformation matrix
   *
   * @return roll,pitch,yaw
   */
  Matrix<3, 1, double> getRPY();

  /**
   * Sets roll, pitch, yaw and calculates tranformation matrix
   *
   * @param roll - roll
   * @param pitch - pitch
   * @param yaw - yaw
   */
  void setRPY(double roll, double pitch, double yaw);

  /**
   * Sets roll, pitch, yaw from matrix values
   * and calculates tranformation matrix
   *
   * @param rpy - matrix with roll, pitch, yaw
   */
  void setRPY(Matrix<3, 1, double> rpy);

  /**
   * Calculates rotational matrix from roll, pitch, yaw
   */
  void calcRotFromRPY();

  /**
   * Calculates roll, pitch, yaw from rotational matrix
   */
  void calcRPYfromRot();

  /**
   * ????
   *
   * @param isRelationFixed - ?
   *
   * @return jacobian
   */
  virtual Matrix<6, 6, double> getJacobiMatrix(bool isRelationFixed = true) {
    Matrix<6, 6, double> jacobian;
    jacobian.zero();
    Matrix<3, 1, double> crossP;
    Matrix<3, 1, double> transAxRef({get(0, 3), get(1, 3), get(2, 3)});
    Matrix<3, 1, double> rotCol;
    for (int i = 0; i < 3; i++) {  // column
      rotCol = Matrix<3, 1, double>({get(i, 0), get(i, 1), get(i, 2)});
      crossP = TF_Matrix::crossProduct(
          rotCol, -transAxRef);              // r x rot (negative direction)
      for (int j = 0; j < 3; j++) {          // row
        jacobian(j, i) = get(j, i);          // upper left
        jacobian(j + 3, i + 3) = get(j, i);  // lower right
        if (isRelationFixed) {
          jacobian(j, i + 3) = crossP(j);  // upper right
        }
      }
    }
    return jacobian;
  }

  /**
   * ????
   *
   * @param tf - ?
   * @param isRelationFixed - ?
   *
   * @return jacobian
   */
  static Matrix<6, 6, double> getJacobiMatrix(Matrix<4, 4, double> tf,
                                              bool isRelationFixed = true) {
    Matrix<6, 6, double> jacobian;
    jacobian.zero();
    Matrix<3, 1, double> crossP;
    Matrix<3, 1, double> transAxRef({tf(0, 3), tf(1, 3), tf(2, 3)});
    Matrix<3, 1, double> rotCol;
    for (int i = 0; i < 3; i++) {  // column
      rotCol = Matrix<3, 1, double>({tf(i, 0), tf(i, 1), tf(i, 2)});
      crossP = TF_Matrix::crossProduct(
          rotCol, -transAxRef);             // r x rot (negative direction)
      for (int j = 0; j < 3; j++) {         // row
        jacobian(j, i) = tf(j, i);          // upper left
        jacobian(j + 3, i + 3) = tf(j, i);  // lower right
        if (isRelationFixed) {
          jacobian(j, i + 3) = crossP(j);  // upper right
        }
      }
    }
    return jacobian;
  }

  /**
   * Invertes the matrix.
   * If the determinant is 0, it will return the eye matrix
   *
   * @return inverse of the matrix
   */
  virtual Matrix<4, 4, double> inv();

 private:
  double roll, pitch, yaw;
};

}  // namespace tf
}  // namespace math
}  // namespace eeros
