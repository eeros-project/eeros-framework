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
  Matrix<3, 1, double> getTrans() const;

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
  Matrix<3, 3, double> getRot() const;

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
  const Matrix<4, 4, double> getMatrix() const;

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
   * Gets roll, pitch, yaw.
   * Make sure to calculate these values beforehand from the
   * transformation matrix using \ref calcRPYfromRot.
   *
   * @return roll,pitch,yaw
   */
  Matrix<3, 1, double> getRPY() const;

  /**
   * Calculates rotational matrix from roll, pitch, yaw
   */
  void calcRotFromRPY();

  /**
   * Calculates roll, pitch, yaw from rotational matrix
   */
  void calcRPYfromRot();

  /**
   * Get the adjoint representation of a transformation matrix.
   * This 6x6 matrix can be used to translate a twist vector.
   *
   * @param isRelationFixed - ?
   *
   * @return adjoint representation
   */
  virtual Matrix<6, 6, double> getAdjointRep(bool isRelationFixed = true);

  /**
   * Get the adjoint representation of a transformation matrix.
   * This 6x6 matrix can be used to translate a twist vector.
   *
   * @param tf - transformation matrix
   * @param isRelationFixed - ?
   *
   * @return adjoint representation
   */
  static Matrix<6, 6, double> getAdjointRep(Matrix<4, 4, double> tf,
                                              bool isRelationFixed = true);

  /**
   * Invertes the transformation matrix.
   * If the determinant is 0, it will return the eye matrix
   *
   * @return inverse of the matrix
   */
  virtual Matrix<4, 4, double> inverse() const;

  /**
   * Prints transformation matrix in the form of
   * Trans: [x y z]' RPY: [r p y]' "
   * Make sure
   */
  virtual void print(std::ostream& os) const;

 private:
  double roll, pitch, yaw;
};

}  // namespace tf
}  // namespace math
}  // namespace eeros
