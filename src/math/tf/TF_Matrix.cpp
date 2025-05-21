#include <eeros/math/tf/TF_Matrix.hpp>

using namespace eeros::math::tf;

void TF_Matrix::setTrans(Vector3 trans) {
  this->set(0, 3, trans(0));
  this->set(1, 3, trans(1));
  this->set(2, 3, trans(2));
}

Matrix<3, 1, double> TF_Matrix::getTrans() const {
  Matrix<3, 1, double> trans;
  trans = {get(0, 3), get(1, 3), get(2, 3)};
  return trans;
}

void TF_Matrix::setRot(Matrix<3, 3, double> rot) {
  this->set(0, 0, rot(0, 0));
  this->set(0, 1, rot(0, 1));
  this->set(0, 2, rot(0, 2));
  this->set(1, 0, rot(1, 0));
  this->set(1, 1, rot(1, 1));
  this->set(1, 2, rot(1, 2));
  this->set(2, 0, rot(2, 0));
  this->set(2, 1, rot(2, 1));
  this->set(2, 2, rot(2, 2));
}

Matrix<3, 3, double> TF_Matrix::getRot() const {
  return getSubMatrix<3, 3>(0, 0);
}

void TF_Matrix::setMatrix(Matrix<4, 4, double> m) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      set(i, j, m.get(i, j));
    }
  }
}

const Matrix<4, 4, double> TF_Matrix::getMatrix() const {
  Matrix<4, 4, double> mat;
  mat = *this;
  return mat;
}

void TF_Matrix::setRPY(double roll, double pitch, double yaw) {
  this->roll = roll;
  this->pitch = pitch;
  this->yaw = yaw;
  calcRotFromRPY();
}

void TF_Matrix::setRPY(Matrix<3, 1, double> rpy) {
  roll = rpy(0);
  pitch = rpy(1);
  yaw = rpy(2);
  calcRotFromRPY();
}

Matrix<3, 1, double> TF_Matrix::getRPY() const {
  return Matrix<3, 1, double>(roll, pitch, yaw);
}

void TF_Matrix::calcRotFromRPY() {
  Matrix<3, 3, double> mat, matx, maty, matz;
  mat.eye();
  matx.rotx(roll);
  maty.roty(pitch);
  matz.rotz(yaw);
  mat = matz * maty * matx;
  this->setRot(mat);
}

void TF_Matrix::calcRPYfromRot() {
  // ! same calculation exists in Quaternion::getRPY() -> keep both updated
  // ! returns values from 0 to PI for yaw

  pitch = -std::asin(get(2, 0));
  if (get(2, 0) == 1 || get(2, 0) == -1) {  // gimbal lock
    eeros::logger::Logger::getLogger().warn()
        << "angle beta is pi/-pi an RPY cannot be calculated!";
  } else {
    // yaw: acos([0,0] / cos(pitch)); check if [1,0] < 0.0 -> yaw is negativ
    yaw = std::acos(get(0, 0) / std::cos(pitch));
    if (get(1, 0) < 0.0) yaw *= -1;
    // roll: asin( [2,1] /cos(beta));
    roll = std::asin(get(2, 1) / std::cos(pitch));
    //  check if [2,2]  is negativ, then roll is in the next quadrant -> roll
    //  > PI (roll > 0 )  or roll < -PI (roll < 0)
    if (get(2, 2) < 0.0 && roll > 0.0) roll = M_PI - roll;
    if (get(2, 2) < 0.0 && roll < 0.0) roll = -(M_PI + roll);
  }
}

Matrix<6, 6, double> TF_Matrix::getAdjointRep(bool isRelationFixed) {
  Matrix<6, 6, double> ad;
  ad.zero();
  Matrix<3, 1, double> crossP;
  Matrix<3, 1, double> transAxRef({get(0, 3), get(1, 3), get(2, 3)});
  Matrix<3, 1, double> rotCol;
  for (int i = 0; i < 3; i++) {  // column
    rotCol = Matrix<3, 1, double>({get(i, 0), get(i, 1), get(i, 2)});
    crossP = TF_Matrix::crossProduct(
        rotCol, -transAxRef);        // r x rot (negative direction)
    for (int j = 0; j < 3; j++) {    // row
      ad(j, i) = get(j, i);          // upper left
      ad(j + 3, i + 3) = get(j, i);  // lower right
      if (isRelationFixed) {
        ad(j, i + 3) = crossP(j);  // upper right
      }
    }
  }
  return ad;
}

Matrix<6, 6, double> TF_Matrix::getAdjointRep(Matrix<4, 4, double> tf,
                                              bool isRelationFixed) {
  Matrix<6, 6, double> ad;
  ad.zero();
  Matrix<3, 1, double> crossP;
  Matrix<3, 1, double> transAxRef({tf(0, 3), tf(1, 3), tf(2, 3)});
  Matrix<3, 1, double> rotCol;
  for (int i = 0; i < 3; i++) {  // column
    rotCol = Matrix<3, 1, double>({tf(i, 0), tf(i, 1), tf(i, 2)});
    crossP = TF_Matrix::crossProduct(
        rotCol, -transAxRef);       // r x rot (negative direction)
    for (int j = 0; j < 3; j++) {   // row
      ad(j, i) = tf(j, i);          // upper left
      ad(j + 3, i + 3) = tf(j, i);  // lower right
      if (isRelationFixed) {
        ad(j, i + 3) = crossP(j);  // upper right
      }
    }
  }
  return ad;
}

Matrix<4, 4, double> TF_Matrix::inverse() const {
  if (det() != 0) {
    return Matrix::inverse();
  } else {
    Matrix<4, 4, double> inv;
    inv.eye();
    return inv;
  }
}

void TF_Matrix::print(std::ostream& os) const {
  os << "Trans: " << getTrans() << "RPY: " << getRPY();
}
