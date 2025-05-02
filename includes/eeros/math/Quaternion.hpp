#pragma once

#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
#include <sstream>
#include <stdexcept>

namespace eeros {
namespace math {

enum QUAT { w, x, y, z };

class Quaternion {
 public:
  Quaternion() {
    x=0.0; y=0.0; z=0.0; w=1.0;
  }

  Quaternion(double _w, double _x, double _y, double _z)
      : w(_w), x(_x),y(_y),z(_z) { }

  Quaternion(Matrix<4, 1, double> m)
      : w(m(0)), x(m(1)), y(m(2)), z(m(3)) { }

  Quaternion(double roll, double pitch, double yaw) {
    setFromRPY(roll, pitch, yaw);
  }

  Quaternion(eeros::math::Matrix<3, 1, double> angles) {
    setFromRPY(angles);
  }

  Matrix<4, 1, double> get() {
    return Matrix<4,1,double>(w, x, y, z);
  }

  void set(double _w, double _x, double _y, double _z) {
    x = _x; y = _y; z = _z; w =_w;
  }

  void set(Matrix<4, 1, double> m) {
    w = m(0); x = m(1); y = m(2); z = m(3);
  }

  Quaternion operator+(const Quaternion quat) {
    Quaternion tmp;
    tmp.w = w + quat.w;
    tmp.x = x + quat.x;
    tmp.y = y + quat.y;
    tmp.z = z + quat.z;
    return tmp;
  }

  Quaternion operator+=(const Quaternion quat) {
    (*this) = (*this) + quat;
    return (*this);
  }

  Quaternion operator-(const Quaternion quat) {
    Quaternion tmp;
    tmp.w = w - quat.w;
    tmp.x = x - quat.x;
    tmp.y = y - quat.y;
    tmp.z = z - quat.z;
    return tmp;
  }

  Quaternion operator-=(const Quaternion quat) {
    (*this) = (*this) - quat;
    return (*this);
  }

  Quaternion operator*(const double f) {
    Quaternion tmp( (w * f), (x * f), (y * f), (z * f));
    return tmp;
  }

  Quaternion operator*(const Quaternion quat) {
    Quaternion tmp;
    tmp.w = (w * quat.w) - (x * quat.x) - (y * quat.y) - (z * quat.z);
    tmp.x = (w * quat.x) + (x * quat.w) + (y * quat.z) - (z * quat.y);
    tmp.y = (w * quat.y) + (y * quat.w) + (z * quat.x) - (x * quat.z);
    tmp.z = (w * quat.z) + (z * quat.w) + (x * quat.y) - (y * quat.x);
    return tmp;
  }

  Quaternion operator*=(const double f) {
    (*this) = (*this) * f;
    return (*this);
  }

  Quaternion operator*=(const Quaternion quat) {
    (*this) = (*this) * quat;
    return (*this);
  }

  Quaternion operator/(const double f) {
    if (f != 0.0) {
      Quaternion tmp((w / f), (x / f), (y / f), (z / f));
      return tmp;
    } else {
      throw Fault("Quaternion: division by zero");
    }
  }

  Quaternion operator/=(const double f) {
    (*this) = (*this) / f;
    return (*this);
  }

  void normalize() {
    double mag = sqrt(x*x + y*y + z*z + w*w);
    x = x / mag;
    y = y / mag;
    z = z / mag;
    w = w / mag;
  }

  Quaternion conj() {
    Quaternion tmp;
    tmp.w = w; tmp.x = -x; tmp.y = -y; tmp.z = -z;
    return tmp;
  }

  Quaternion inv() {
     return conj() / std::pow(len(),2);
  }

  double len() {
    return sqrt(x*x + y*y + z*z + w*w);
  }

  void setFromRot(Matrix<3, 3, double> rot) {
    double trace = rot(0,0) + rot(1,1) + rot(2,2);
    if( trace > 0 ) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      w = 0.25 / s;
      x = (rot(2,1) - rot(1,2)) * s;
      y = (rot(0,2) - rot(2,0)) * s;
      z = (rot(1,0) - rot(0,1)) * s;
    } else {
      if ( rot(0,0) > rot(1,1) && rot(0,0) > rot(2,2) ) {
        double s = 2.0 * std::sqrt( 1.0 + rot(0,0) - rot(1,1) - rot(2,2));
        w = (rot(2,1) - rot(1,2)) / s;
        x = 0.25 * s;
        y = (rot(0,1) + rot(1,0)) / s;
        z = (rot(0,2) + rot(2,0)) / s;
      } else if (rot(1,1) > rot(2,2)) {
        float s = 2.0* std::sqrt(1.0 + rot(1,1) - rot(0,0) - rot(2,2));
        w = (rot(0,2) - rot(2,0)) / s;
        x = (rot(0,1) + rot(1,0)) / s;
        y = 0.25 * s;
        z = (rot(1,2) + rot(2,1)) / s;
      } else {
        float s = 2.0 * std::sqrt( 1.0 + rot(2,2) - rot(0,0) -rot(1,1));
        w = (rot(1,0) - rot(0,1)) / s;
        x = (rot(0,2) + rot(2,0)) / s;
        y = (rot(1,2) + rot(2,1)) / s;
        z = 0.25 * s;
      }
    }
  }

  void setFromRPY(Matrix<3, 1, double> rpy) {
    setFromRPY(rpy(0), rpy(1), rpy(2));
  }

  void setFromRPY(double roll, double pitch, double yaw) {
    Matrix<3,3,double> m, mx, my, mz;
    m.eye();
    mx.rotx(roll);
    my.roty(pitch);
    mz.rotz(yaw);
    m= mz*my*mx;
    setFromRot(m);
  }

  Matrix<3, 1, double> getRPY() {
    Matrix<3,1,double> angle;
    Matrix<3,3,double> mat = getRot();
    // beta
    angle(1)= -std::asin(mat(2,0) );
    if(mat(2,0) == 1 || mat(2,0) == -1) {       // gimbal lock
      eeros::logger::Logger::getLogger().warn() << "Quaternion: angle beta is pi/-pi an RPY cannot be calculated! ";
    } else {
      // gamma: acos([0,0] /cos(beta)); check if [1,0] < 0.0 -> gamma is negativ
      angle(2)= std::acos(mat(0,0)/std::cos(angle(1)));
      if (mat(1,0) < 0.0) angle(2) *= -1;
      // alpha: asin( [2,1] /cos(beta));
      angle(0)= std::asin(mat(2,1)/std::cos(angle(1)));
      //  check if [2,2]  is negativ, then alpha is in the next quadrant -> alpha > PI (alpha > 0 )  or alpha < -PI (alpha< 0)
      if( mat(2,2) < 0.0 && angle[0] > 0.0 ) angle[0] = M_PI- angle[0];
      if( mat(2,2) < 0.0 && angle[0] < 0.0 ) angle[0] =  -(M_PI+ angle[0]) ;
    }
    return angle;
  }

  Matrix<3, 3, double> getRot() {
    Matrix<3,3,double> rot;
    rot(0,0) = 1- 2*(y*y) - 2*(z*z); rot(0,1) = 2*x*y - 2* z*w;      rot(0,2) = 2*x*z + 2*y*w;
    rot(1,0) = 2*x*y + 2*z*w;        rot(1,1) = 1- 2*(x*x)- 2*(z*z); rot(1,2) = 2*y*z - 2*x*w;
    rot(2,0) = 2*x*z - 2*y*w;        rot(2,1) = 2*y*z +2*x*w;        rot(2,2) = 1-2*(x*x) - 2*(y*y);
    return rot;
  }

 public:
  double w; // real
  double x; // complex
  double y;
  double z;
};

/********** Print functions **********/

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
  os << "real w: " << q.w << ", vec: [ " << q.x << " , " << q.y << " , " << q.z << " ]' ";
  return os;
}

}
}
