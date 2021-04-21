#ifndef ORG_EEROS_CONTROL_PATHPLANNERCONSTACC_HPP_
#define ORG_EEROS_CONTROL_PATHPLANNERCONSTACC_HPP_

#include <eeros/control/Output.hpp>
#include <eeros/control/TrajectoryGenerator.hpp>
#include <eeros/core/System.hpp>
#include <cmath>
#include <mutex>

namespace eeros {
namespace control {

/**
 * This path planner with constant acceleration generates a trajectory as follows.
 * The trajectory leads from the start position to its end position. The acceleration is 
 * set to a positive constant value. The velocity starts from 0 and goes up to its maximum value
 * which can be chosen. After this maximum velocity is reached the acceleration is set to 0 and
 * the trajectory continues with constant velocity. Towards the end a constant deceleration
 * makes sure that the final position is reached with the velocity reaching 0. 
 * 
 * @tparam T - output type (must be a composite type), 
 *             a trajectory in 3-dimensional space needs T = Matrix<3,1,double>, 
 *             a trajectory in linear space needs T = Matrix<1,1,double>
 *
 * @since v1.0
 */

template<typename T>
class PathPlannerConstAcc : public TrajectoryGenerator<T, 3> {
  using E = typename T::value_type;
  
 public:
  /**
   * Constructs a path planner with a trajectory where the acceleration is always constant. 
   * The trajectory will start with an constant acceleration (will not be higher than accMax).
   * As soon as velMax is reached, the acceleration will be 0 and the velocity will stay constant 
   * (will not be higher than velMax). For the third part the trajectory will stop with a 
   * constant deceleration (will not be higher decMax).
   * The sampling time must be set to the time with which the timedomain containing this block will run.
   *
   * @param velMax - maximum velocity
   * @param acc - maximum acceleration
   * @param dec - maximum deceleration
   * @param dt - sampling time
   */
  PathPlannerConstAcc(T velMax, T acc, T dec, double dt) 
      : finished(true), velMax(velMax), acc(acc), dec(dec), dt(dt) { 
    posOut.getSignal().clear();
    velOut.getSignal().clear();
    accOut.getSignal().clear();
  }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  PathPlannerConstAcc(const PathPlannerConstAcc& s) = delete; 

  /**
   * Query if a requested trajectory has already reached its end position.
   *
   * @return - end of trajectory is reached
   */
  virtual bool endReached() {
    std::lock_guard<std::mutex> lck(mtx);
    return finished;
  }
  
  /**
   * Runs the path planner block. With the precalculated values for the 
   * acceleration for all sampling points on the trajectory, the resulting velocities
   * and positions are calculated and written to the appropriate outputs.
   */
  virtual void run() {
    std::lock_guard<std::mutex> lck(mtx);
    std::array<T, 3> y = this->last;
    t += dt;
    
    if (!finished) {
      for (unsigned int i = 0; i < a1p.size(); i++) {
        switch (range) {
          case 1:
            if (t <= dT1) {
              y[0][i] = a1p[i] * pow(t, 2) + c1p[i];
              y[1][i] = b1v[i] * t;
              y[2][i] = c1a[i];
            } 
            if (fabs(t - dT1) < 1e-12 && i == a1p.size() - 1) {
              range = 2;
              t = 0;
            }
            break;
          case 2:
            if (t <= dT2) {
              y[0][i] = b2p[i] * t + c2p[i];
              y[1][i] = c2v[i];
              y[2][i] = c2a[i];
            }
            if (dT2 < 1e-12 || (fabs(t - dT2) < 1e-12 && i == a1p.size() - 1)) {
              range = 3;
              t = 0;
            }
            break;
          case 3:
            if (t <= dT3) {
              y[0][i] = a3p[i] * pow(t, 2) + b3p[i] * t + c3p[i];
              y[1][i] = b3v[i] * t + c3v[i];
              y[2][i] = c3a[i];
            }
            if (fabs(t - dT3) < 1e-12 && i == a1p.size() - 1) {
              range = 4;
              t = 0;
            }
            break;
          case 4:
            finished = true;
            y[0][i] = endPos[i];
            y[1][i] = 0.0;
            y[2][i] = 0.0;
            break;
        }
        // keep last position value
        this->last[0][i] = y[0][i];
        this->last[1][i] = y[1][i];
        this->last[2][i] = y[2][i];
      }
    }

    posOut.getSignal().setValue(y[0]);
    velOut.getSignal().setValue(y[1]);
    accOut.getSignal().setValue(y[2]);

    timestamp_t time = System::getTimeNs(); 
    posOut.getSignal().setTimestamp(time);
    velOut.getSignal().setTimestamp(time);
    accOut.getSignal().setTimestamp(time);
  }
  
  using TrajectoryGenerator<T, 3>::move;
  
  /**
   * Dispatches a new trajectory from start to end.
   * With both parameters only the first index in the array, that is the position
   * will be considered. The higher derivatives won't be taken into account.
   * The function will terminate immediately and return false if the last trajectory 
   * is not finished.
   * 
   * @param start - array containing start position and its higher derivatives
   * @param end - array containing end position and its higher derivatives
   * @return - a trajectory could be sucessfully generated for this parameters
   *
   * @see setStart()
   * @see run()
   */
  virtual bool move(std::array<T, 3> start, std::array<T, 3> end) {
    if (!finished) return false;
    T calcVelNorm, calcAccNorm, calcDecNorm;
    E velNorm, accNorm, decNorm;
    T distance = end[0] - start[0];
    endPos = end[0];
    
    T zero; zero = 0;
    if (distance == zero) return false;
    
    // normalize 
    for (unsigned int i = 0; i < calcVelNorm.size(); i++) {
      calcVelNorm[i] = fabs(velMax[i] / distance[i]);
      calcAccNorm[i] = fabs(acc[i] / distance[i]);
      calcDecNorm[i] = fabs(dec[i] / distance[i]);
    }
    
    // find minimum
    velNorm = calcVelNorm[0]; accNorm = calcAccNorm[0]; decNorm = calcDecNorm[0]; 
    for (unsigned int i = 0; i < calcVelNorm.size(); i++) {
      if (calcVelNorm[i] < velNorm) velNorm = calcVelNorm[i];
      if (calcAccNorm[i] < accNorm) accNorm = calcAccNorm[i];
      if (calcDecNorm[i] < decNorm) decNorm = calcDecNorm[i];
    }
    
    // determine if maximum velocity can be reached, else limit it
    E velNormMax = sqrt(2 * (accNorm * decNorm) / (accNorm + decNorm));
    if (velNorm > velNormMax) velNorm = velNormMax; 
    
    // calculate time intervals    
    dT1 = velNorm / accNorm;
    dT3 = velNorm / decNorm;
    dT2 = 1 / velNorm - (dT1 + dT3) * 0.5;
    if (dT2 < 0) dT2 = 0;
   
    // make time intervals multiple of sampling time
    dT1 = ceil(dT1 / dt) * dt;
    dT2 = ceil(dT2 / dt) * dt;
    dT3 = ceil(dT3 / dt) * dt; 
//     log.info() << "dT1 = " << dT1 << ", dT2 = " << dT2 << ", dT3 = " << dT3;
  
    // recalculate velocity with definitive time interval values
    velNorm = 1 / ((dT2 + (dT1 + dT3) / 2));
//     log.info() << "vel norm = " << velNorm;
    
    T vel = velNorm * distance;
    c1a = vel / dT1;
    b1v = c1a;
    a1p = 0.5 * c1a;
    c1p = start[0];
     
    c2a = 0; 
    c2v = vel;
    b2p = c2v;
    c2p = a1p * pow(dT1,2) + c1p;
    
    c3a = -vel / dT3;
    b3v = c3a;  
    c3v = vel; 
    a3p = 0.5 * c3a;
    b3p = c3v;
    c3p = b2p * dT2 + c2p;
    
    std::lock_guard<std::mutex> lck(mtx);
    finished = false;
    range = 1;
    t = 0;  
    return true;
  }
  
  using TrajectoryGenerator<T, 3>::setStart;

  /**
   * Sets the start position and state of the higher derivatives (velocity, acceleration ...)
   * 
   * @param start - array containing start position and its higher derivatives
   */
  virtual void setStart(std::array<T, 3> start) {
    std::lock_guard<std::mutex> lck(mtx);
    this->last = start;
    this->finished = true;
  }
  
  /**
   * Sets the maximum value for the velocity. The maximum velocity during the steady phase of the trajectory.
   *
   * @param vel - maximum velocity
   */
  virtual void setMaxSpeed(T vel) {velMax = vel;}
  
  /**
   * Sets the value for the acceleration. The acceleration is used for the start of the trajectory.
   *
   * @param acc - acceleration
   */
  virtual void setMaxAcc(T acc) {this->acc = acc;}
  
  /**
   * Sets the value for the deceleration. The deceleration is used for the end of the trajectory.
   *
   * @param dec - deceleration
   */
  virtual void setMaxDec(T dec) {this->dec = dec;}
  
  /**
   * Getter function for the position output.
   * 
   * @return The position output
   */
  virtual Output<T>& getPosOut() {return posOut;}
  
  /**
   * Getter function for the velocity output.
   * 
   * @return The velocity output
   */
  virtual Output<T>& getVelOut() {return velOut;}
  
  /**
   * Getter function for the acceleration output.
   * 
   * @return The acceleration output
   */
  virtual Output<T>& getAccOut() {return accOut;}
  
 private:
  Output<T> posOut, velOut, accOut;
  bool finished;
  int range;
  T velMax, acc, dec;
  double t, dt, dT1, dT2, dT3;
  // naming of coefficients: a|b|c & 1|2|3 & a|v|p
  // a|b|c : a for t*t factor, b for linear factor, c for constant
  // 1|2|3 : one of the three parts on the time axis
  // a|v|p : result will be a for acc, v for vel, p for position
  // e.g. a1p -> coeffizient for time intervall 1, used to multiply with t^2 resulting in the position
  T a1p, c1p, b1v, c1a, b2p, c2p, c2v, c2a, a3p, b3p, c3p, b3v, c3v, c3a; 
  T endPos;
  std::mutex mtx;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * PathPlannerConstAcc instance to an output stream.\n
 * Does not print a newline control character.
 */
template<typename T>
std::ostream &operator<<(std::ostream &os, PathPlannerConstAcc<T> &pp) {
  os << "Block path planner constant acceleration: '" << pp.getName() << "'";
  return os;
}

};
};

#endif /* ORG_EEROS_CONTROL_PATHPLANNERCONSTACC_HPP_ */

