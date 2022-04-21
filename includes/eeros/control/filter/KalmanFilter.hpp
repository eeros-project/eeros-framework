#ifndef ORG_EEROS_CONTROL_KALMANFILTER_HPP_
#define ORG_EEROS_CONTROL_KALMANFILTER_HPP_

#include <eeros/core/System.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>
#include <eeros/math/Matrix.hpp>
#include <mutex>

using namespace eeros::math;

namespace eeros {
namespace control {

/**
 * Helper class to separate prediction and correction of the kalman filter.
 * This class is used to periodically run the prediction in a timedomain.
 */
template <uint8_t nofInputs, uint8_t nofOutputs, uint8_t nofStates, uint8_t nofRandVars>
class KalmanFilterPrediction;

/**
 * Helper class to separate prediction and correction of the kalman filter.
 * This class is used to periodically run the correction in a timedomain.
 */
template <uint8_t nofInputs, uint8_t nofOutputs, uint8_t nofStates, uint8_t nofRandVars>
class KalmanFilterCorrection;

/**
 * A kalman filter block is used to estimate the state of a system. 
 * The system must be described in a state space representation:
 * 
 * u -----------------------> physical system ------------------------------> y
 *     |                                                              |
 *     |                                                              |
 *     |   ---------------------> D ---------------------             |
 *     |  |                                              |            |
 *     |  |                                              | +          | -
 *     |  |         +      x(k-1)          x(k)       +  v   y_hat +  v
 *      -----> Bd ---> 0 ----------> z⁻1 -------> C ---> 0 ---------> 0
 *                     ^                                 |            |
 *                   + |                                 | +          |
 *                     |                                 v            |
 *                      -------------- Ad <------------- 0 <--- K ----
 *                                                          +
 *
 * with x - state vector
 *      u - input vector of the physical system
 *      y - output vector of the physical system
 *      
 *      Ad - time discrete system matrix
 *      Bd - time discrete input matrix
 *      C - output matrix
 *      D - feed forward matrix
 *      K - kalman feedback matrix
 * 
 * Furthermore, disturbances in the system need to be modeled as noise in a separate 
 * vector z with a related covariance matrix G. Measurement noise is added to the
 * output by introducing a vector v.
 * 
 * The complete state space model is
 *          x(k-1) = Ad*x(k) + Bd*u(k) + Gd*z(k)
 *          y(k) = C*x(k) + D*u(k) + v(k)
 * 
 * For a detailed introduction to the topic, please refer to the book 
 * "Kalman-Filter Einführung in die Zustandsschätzung und ihre Anwendungen für 
 * eingebettete Systeme" by Reiner Marchthaler and Sebastian Dingler.
 * 
 * The kalman filter block has two separate blocks included. One is for the 
 * prediction and the other one for the correction. The correction block
 * should be run after reading the sensor values, while the prediction block should 
 * run after the input vector is defined. The two blocks can run in different time domains.
 * 
 * @tparam nofInputs - number of system inputs
 * @tparam nofOutputs - number of system outputs
 * @tparam nofStates - number of states
 * @tparam nofRandVars - number of random variables to model the system noise
 * 
 * @since v1.2
 */
template <uint8_t nofInputs, uint8_t nofOutputs, uint8_t nofStates, uint8_t nofRandVars>
class KalmanFilter : public Blockio<0,0>{
 public:
  
  /**
   * Constructs a kalman filter instance by providing the matrices Ad, Bd, C, Gd, Q and R.
   * The matrix D is set to 0 (no feed forward path).
   * The initial state of the system x is set to 0.
   * The initial covariance of the estimation error is set to 0.
   *
   * @param Ad - time discrete system matrix
   * @param Bd - time discrete input matrix
   * @param C - output matrix
   * @param Gd - time discrete kovariance matrix of the system disturbance
   * @param Q - variance of the system noise
   * @param R - variance of the process noise
   */
  KalmanFilter(Matrix<nofStates, nofStates> Ad,
               Matrix<nofStates, nofInputs> Bd,
               Matrix<nofOutputs, nofStates> C,
               Matrix<nofStates, nofRandVars> Gd,
               Matrix<nofRandVars, nofRandVars> Q,
               Matrix<nofOutputs, nofOutputs> R)
        : Ad(Ad), Bd(Bd), C(C), Gd(Gd), Q(Q), R(R), predict(this), correct(this) {
    this->D.zero();
    this->x.zero();
    this->P.eye();
    this->eye.eye();
    this->GdQGdT = Gd * Q * Gd.transpose();
  }
  
  /**
   * Constructs a kalman filter instance by providing the matrices Ad, Bd, C, D, Gd, Q and R.
   * The initial state of the system x is set to 0.
   * The initial covariance of the estimation error is set to 0.
   *
   * @param Ad - time discrete system matrix
   * @param Bd - time discrete input matrix
   * @param C - output matrix
   * @param D - feed forward matrix
   * @param Gd - time discrete kovariance matrix of the system disturbance
   * @param Q - variance of the system noise
   * @param R - variance of the process noise
   */
  KalmanFilter(Matrix<nofStates, nofStates> Ad,
               Matrix<nofStates, nofInputs> Bd,
               Matrix<nofOutputs, nofStates> C,
               Matrix<nofOutputs, nofInputs> D,
               Matrix<nofStates, nofRandVars> Gd,
               Matrix<nofRandVars, nofRandVars> Q,
               Matrix<nofOutputs, nofOutputs> R)
      : Ad(Ad), Bd(Bd), C(C), D(D), Gd(Gd), Q(Q), R(R), predict(this), correct(this) {
    this->x.zero();
    this->P.eye();
    this->eye.eye();
    this->GdQGdT = Gd * Q * Gd.transpose();
  }
    
  /**
   * Constructs a kalman filter instance by providing the matrices Ad, Bd, C, D, Gd, Q, R and P and the vector x.
   *
   * @param Ad - time discrete system matrix
   * @param Bd - time discrete input matrix
   * @param C - output matrix
   * @param D - feed forward matrix
   * @param Gd - time discrete kovariance matrix of the system disturbance
   * @param Q - variance of the system noise
   * @param R - variance of the process noise
   * @param P - initial covariance of the estimation error
   * @param x - initial state of the system
   */
  KalmanFilter(Matrix<nofStates, nofStates> Ad,
               Matrix<nofStates, nofInputs> Bd,
               Matrix<nofOutputs, nofStates> C,
               Matrix<nofOutputs, nofInputs> D,
               Matrix<nofStates, nofRandVars> Gd,
               Matrix<nofRandVars, nofRandVars> Q,
               Matrix<nofOutputs, nofOutputs> R,
               Matrix<nofStates, nofStates> P,
               Vector<nofStates> x)
      : Ad(Ad), Bd(Bd), C(C), D(D), Gd(Gd), Q(Q), R(R), P(P), x(x), predict(this), correct(this) {
    this->eye.eye();
    this->GdQGdT = Gd * Q * Gd.transpose();
  }

  /**
   * Get vector element with index index of system input vector u.
   * 
   * @param index - element index
   */
  Input<double> &getU(uint8_t index) {
    if (index < 0 || index >= nofInputs) {
      throw eeros::control::IndexOutOfBoundsFault("Trying to get inexistent element of system input vector u in Block " +
                                                      this->getName() + ".");
    }
    return inU[index];
  }

  /**
   * Get vector element with index index of system output vector y.
   * 
   * @param index - element index
   */
  Input<double> &getY(uint8_t index) {
    if (index < 0 || index >= nofOutputs) {
      throw eeros::control::IndexOutOfBoundsFault("Trying to get inexistent element of system output vector y in Block " +
                                                        this->getName() + ".");
    }
    return inY[index];
  }

  /**
   * Get vector element with index index of system state vector x.
   * 
   * @param index - element index
   */
  Output<double> &getX(uint8_t index) {
    if (index < 0 || index >= nofStates) {
      throw eeros::control::IndexOutOfBoundsFault("Trying to get inexistent element of system state vector x in Block " +
                                                      this->getName() + ".");
    }
    return out[index];
  }

  /**
   * Predict current system state
   */
  void prediction() {
    std::lock_guard<std::mutex> lock(mtx);
    for (uint8_t i = 0; i < nofInputs; i++)
    {
        u[i] = inU[i].getSignal().getValue();
    }
    x = Ad * x + Bd * u;
    for (uint8_t i = 0; i < nofStates; i++) {
      out[i].getSignal().setValue(x[i]);
      out[i].getSignal().setTimestamp(eeros::System::getTimeNs());
    }
    P = Ad * P * Ad.transpose() + GdQGdT;
  }

  /**
   * Correct current system state
   */
  void correction() {
    std::lock_guard<std::mutex> lock(mtx);
    if (first) {
      for (uint8_t i = 0; i < nofStates; i++) {
        out[i].getSignal().setValue(x[i]);
        out[i].getSignal().setTimestamp(eeros::System::getTimeNs());
      }
      first = false;
    } else {
      for (uint8_t i = 0; i < nofOutputs; i++)
      {
          y[i] = inY[i].getSignal().getValue();
      }
      for (uint8_t i = 0; i < nofInputs; i++)
      {
          u[i] = inU[i].getSignal().getValue();
      }
      CPCTR = (C * P * C.transpose() + R);
      K = P * C.transpose() * !CPCTR;
      dy = y - C * x - D * u;
      x = x + K * dy;
      for (uint8_t i = 0; i < nofStates; i++) {
        out[i].getSignal().setValue(x[i]);
        out[i].getSignal().setTimestamp(eeros::System::getTimeNs());
      }
      P = (eye - K * C) * P;
    }
  }

 protected:
  std::mutex mtx;
  Vector<nofStates> x;
  Vector<nofOutputs> dy;
  Vector<nofOutputs> y;
  Vector<nofInputs> u;
  Input<double> inY[nofOutputs];
  Input<double> inU[nofInputs];
  Output<double> out[nofStates];
  Matrix<nofStates, nofStates> Ad, P, eye, GdQGdT;
  Matrix<nofStates, nofInputs> Bd;
  Matrix<nofStates, nofOutputs> K;
  Matrix<nofOutputs, nofStates> C;
  Matrix<nofOutputs, nofInputs> D;
  Matrix<nofStates, nofRandVars> Gd;
  Matrix<nofRandVars, nofRandVars> Q;
  Matrix<nofOutputs, nofOutputs> R, CPCTR;
  bool first = true;

 public:
  KalmanFilterPrediction<nofInputs, nofOutputs, nofStates, nofRandVars> predict;
  KalmanFilterCorrection<nofInputs, nofOutputs, nofStates, nofRandVars> correct;

 private:
  /**
    * This method is required because the class inherits from eeros::control::Block,
    * which inherits from eeros::Runnable.
    * This run method does not do anything. Use the run methods of the helper classes instead.
    */
  void run(){};
};

template <uint8_t nofInputs, uint8_t nofOutputs, uint8_t nofStates, uint8_t nofRandVars>
class KalmanFilterPrediction : public Block {
 public:
  KalmanFilterPrediction(KalmanFilter<nofInputs, nofOutputs, nofStates, nofRandVars> *owner)
      : owner(owner) {}

  virtual void run() {
    owner->prediction();
  }

 private:
  KalmanFilter<nofInputs, nofOutputs, nofStates, nofRandVars> *owner;
};

template <uint8_t nofInputs, uint8_t nofOutputs, uint8_t nofStates, uint8_t nofRandVars>
class KalmanFilterCorrection : public Block {
 public:
  KalmanFilterCorrection(KalmanFilter<nofInputs, nofOutputs, nofStates, nofRandVars> *owner)
      : owner(owner) {}

  virtual void run() {
    owner->correction();
  }

 private:
  KalmanFilter<nofInputs, nofOutputs, nofStates, nofRandVars> *owner;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * kalman filter instance to an output stream.\n
 * Does not print a newline control character.
 */
template <uint8_t nofInputs, uint8_t nofOutputs, uint8_t nofStates, uint8_t nofRandVars>
std::ostream& operator<<(std::ostream& os, KalmanFilter<nofInputs,nofOutputs,nofStates,nofRandVars>& kf) {
  os << "Block KalmanFilter: '" << kf.getName(); 
  return os;
}

}
}

#endif // ORG_EEROS_CONTROL_KALMANFILTER_HPP_
