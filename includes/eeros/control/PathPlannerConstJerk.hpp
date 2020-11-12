#ifndef ORG_EEROS_CONTROL_PATHPLANNERCONSTJERK_HPP_
#define ORG_EEROS_CONTROL_PATHPLANNERCONSTJERK_HPP_

#include <eeros/control/Output.hpp>
#include <eeros/control/TrajectoryGenerator.hpp>
#include <eeros/core/System.hpp>
#include <cmath>
#include <mutex>

namespace eeros {
namespace control {

/**
 * This path planner with constant jerk generates a trajectory as follows.
 * The trajectory leads from the start position to its end position. The jerk is 
 * set to a positive constant value. The acceleration starts from 0 and goes up to its maximum value.
 * The jerk is then set to a negative value until the acceleration reaches 0 again. 
 * This causes the velocity to reach its maximum value which can be chosen. 
 * Towards the end the procedure is repeated with a negative jerk followed by a positive jerk.
 * This ensures that the final position is reached with the velocity reaching 0. 
 * 
 * @tparam T - output type (must be a composite type), 
 *             a trajectory in 3-dimensional space needs T = Matrix<3,1,double>, 
 *             a trajectory in linear space needs T = Matrix<1,1,double>
 *
 * @since v1.0
 */

template<typename T>
class PathPlannerConstJerk : public TrajectoryGenerator<T, 4> {
  using E = typename T::value_type;
  
 public:
  /**
   * Constructs a path planner with a trajectory where the jerk is constant. 
   * The trajectory will start with an constant jerk (will not be higher than jerk).
   * As soon as accMax is reached, the jerk will be negative and the acceleration will go back to 0.
   * This causes the velocity to reach velMax. Towards the end the procedure is repeated with a 
   * negative jerk followed by a positive jerk. This ensures that the final position is 
   * reached with the velocity reaching 0.
   * The sampling time must be set to the time with which the timedomain containing this block will run.
   *
   * @param velMax - maximum velocity
   * @param accMax - maximum acceleration
   * @param decMax - maximum deceleration
   * @param dt - sampling time
   */
  PathPlannerConstJerk(T velMax, T jerk, double dt) 
      : finished(true), velMax(velMax), jerk(jerk), dt(dt) {
    posOut.getSignal().clear();
    velOut.getSignal().clear();
    accOut.getSignal().clear();
    jerkOut.getSignal().clear();
  }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  PathPlannerConstJerk(const PathPlannerConstJerk& s) = delete; 

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
    std::array<T, 4> y = this->last;
    t += dt;
    
    if (!finished) {
      for (unsigned int i = 0; i < a1p.size(); i++) {
        switch (range) {
          case 1:
            if (t <= dT1) {
              y[0][i] = a1p[i] * pow(t,3) + d1p[i];
              y[1][i] = b1v[i] * pow(t,2);
              y[2][i] = c1a[i] * t;
              y[3][i] = d1j[i];
            } 
            if (fabs(t - dT1) < 1e-12 && i == a1p.size() - 1) {
              range = 2;
              t = 0;
            }
            break;
          case 2:
            if (t <= dT1) {
              y[0][i] = a2p[i] * pow(t,3) + b2p[i] * pow(t,2) + c2p[i] * t + d2p[i];
              y[1][i] = b2v[i] * pow(t,2) + c2v[i] * t + d2v[i];
              y[2][i] = c2a[i] * t + d2a[i];
              y[3][i] = d2j[i];
            }
            if (fabs(t - dT1) < 1e-12 && i == a1p.size() - 1) {
              range = 3;
              t = 0;
            }
            break;
          case 3:
            if (t <= dT2) {
              y[0][i] = c3p[i] * t + d3p[i];
              y[1][i] = d3v[i];
              y[2][i] = 0;
              y[3][i] = 0;
            }
            if (fabs(t - dT2) < 1e-12 && i == a1p.size() - 1) {
              range = 4;
              t = 0;
            }
            break;
          case 4:
            if (t <= dT1) {
              y[0][i] = a4p[i] * pow(t,3) + c4p[i] * t + d4p[i];
              y[1][i] = b4v[i] * pow(t,2) + d4v[i];
              y[2][i] = c4a[i] * t;
              y[3][i] = d4j[i];
            } 
            if (fabs(t - dT1) < 1e-12 && i == a1p.size() - 1) {
              range = 5;
              t = 0;
            }
            break;
          case 5:
            if (t <= dT1) {
              y[0][i] = a5p[i] * pow(t,3) + b5p[i] * pow(t,2) + c5p[i] * t + d5p[i];
              y[1][i] = b5v[i] * pow(t,2) + c5v[i] * t + d5v[i];
              y[2][i] = c5a[i] * t + d5a[i];
              y[3][i] = d5j[i];
            }
            if (fabs(t - dT1) < 1e-12 && i == a1p.size() - 1) {
              range = 6;
              t = 0;
            }
            break;
          case 6:
            finished = true;
            y[0][i] = endPos[i];
            y[1][i] = 0.0;
            y[2][i] = 0.0;
            y[3][i] = 0.0;
            break;
        }
        // keep last position value
        this->last[0][i] = y[0][i];
        this->last[1][i] = y[1][i];
        this->last[2][i] = y[2][i];
        this->last[3][i] = y[3][i];
      }
    }

    posOut.getSignal().setValue(y[0]);
    velOut.getSignal().setValue(y[1]);
    accOut.getSignal().setValue(y[2]);
    jerkOut.getSignal().setValue(y[3]);

    timestamp_t time = System::getTimeNs(); 
    posOut.getSignal().setTimestamp(time);
    velOut.getSignal().setTimestamp(time);
    accOut.getSignal().setTimestamp(time);
    jerkOut.getSignal().setTimestamp(time);
  }
  
  using TrajectoryGenerator<T, 4>::move;
  
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
  virtual bool move(std::array<T, 4> start, std::array<T, 4> end) {
    if (!finished) return false;
    T calcVelNorm, calcJerkNorm;
    E velNorm, jerkNorm;
    T distance = end[0] - start[0];
    endPos = end[0];
   
    T zero; zero = 0;
    if (distance == zero) return false;
    
    // normalize
    for (unsigned int i = 0; i < calcVelNorm.size(); i++) {
      calcVelNorm[i] = fabs(velMax[i] / distance[i]);
      calcJerkNorm[i] = fabs(jerk[i] / distance[i]);
    }
    
    // find minimum
    velNorm = calcVelNorm[0]; jerkNorm = calcJerkNorm[0]; 
    for (unsigned int i = 0; i < calcVelNorm.size(); i++) {
      if (calcVelNorm[i] < velNorm) velNorm = calcVelNorm[i];
      if (calcJerkNorm[i] < jerkNorm) jerkNorm = calcJerkNorm[i];
    }
    
    // determine if maximum velocity can be reached, else limit it
    E velNormMax = 1 / (2 * cbrt(1 / (2 * jerkNorm)));
    if (velNorm > velNormMax) velNorm = velNormMax; 
    
    // calculate time intervals    
    dT1 = sqrt(velNorm / jerkNorm);
    dT2 = 1 / velNorm - 4 * dT1 * 0.5;
    if (dT2 < 0) dT2 = 0;
    
    // make time intervals multiple of sampling time
    dT1 = ceil(dT1 / dt) * dt;
    dT2 = ceil(dT2 / dt) * dt;
//    log.info() << "dT1 = " << dT1 << ", dT2 = " << dT2 << ", total time = " << (4 * dT1 + dT2);
    
    // recalculate velocity with definitive time interval values
    velNorm = 1 / (dT2 + 4 * dT1 * 0.5);
    
    d1j = velNorm * distance / pow(dT1,2);
    c1a = d1j;
    b1v = d1j / 2;
    a1p = d1j / 6;
    d1p = start[0];

    d2j = -d1j;
    c2a = -d1j;
    d2a = d1j * dT1;
    b2v = -d1j / 2;
    c2v = d1j * dT1;
    d2v = d1j / 2 * pow(dT1,2);
    a2p = -d1j / 6;
    b2p = d1j / 2 * dT1;
    c2p = d1j / 2 * pow(dT1,2);
    d2p = d1j / 6 * pow(dT1,3) + d1p;

    d3v = d1j * pow(dT1,2);
    c3p = d1j * pow(dT1,2);
    d3p = d1j * pow(dT1,3) + d1p;
      
    d4j = -d1j;
    c4a = -d1j;
    b4v = -d1j / 2;
    d4v = d1j * pow(dT1,2);
    a4p = -d1j / 6;
    c4p = d1j * pow(dT1,2);
    d4p = d1j * (pow(dT1,2) * dT2 + pow(dT1,3)) + d1p;
    
    d5j = d1j;
    c5a = d1j;
    d5a = -d1j * dT1;
    b5v = d1j / 2;
    c5v = -d1j * dT1;
    d5v = d1j / 2 * pow(dT1,2);
    a5p = d1j / 6;
    b5p = -d1j / 2 * dT1;
    c5p = d1j / 2 * pow(dT1,2);
    d5p = d1j * (pow(dT1,2) * dT2 + 11.0 / 6 * pow(dT1,3)) + d1p;
    
    std::lock_guard<std::mutex> lck(mtx);
    finished = false;
    range = 1;
    t = 0;  
    return true;
  }
  
  using TrajectoryGenerator<T, 4>::setStart;

  /**
   * Sets the start position and state of the higher derivatives (velocity, acceleration ...)
   * 
   * @param start - array containing start position and its higher derivatives
   */
  virtual void setStart(std::array<T, 4> start) {
    std::lock_guard<std::mutex> lck(mtx);
    this->last = start;
    this->finished = true;
  }
  
  /**
   * Sets the maximum value for the velocity. The maximum velocity during the steady phase of the trajectory.
   *
   * @param max - maximum velocity
   */
  virtual void setMaxVel(T max) {velMax = max;}
  
  /**
   * Sets the maximum value for the acceleration. The maximum acceleration is used for the start of the trajectory.
   *
   * @param max - maximum acceleration
   */
  virtual void setJerk(T val) {jerk = val;}
  
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
  
  /**
   * Getter function for the jerk output.
   * 
   * @return The jerk output
   */
  virtual Output<T>& getJerkOut() {return jerkOut;}

 private:
  Output<T> posOut, velOut, accOut, jerkOut;
  bool finished;
  int range;
  T velMax, jerk;
  double t, dt, dT1, dT2;
  // naming of coefficients: a|b|c & 1|2|3 & a|v|p
  // a|b|c|d : a for jerk, b for acc, c for vel, d for position
  // 1|2|3|4|5 : one of the three parts on the time axis
  // j|a|v|p : j for jerk, a for acc, v for vel, p for position
  // e.g. a1p -> coeffizient for time intervall 1, used to multiply with t^2 resulting in the position
  T a1p, d1p, b1v, c1a, d1j, 
    a2p, b2p, c2p, d2p, b2v, c2v, d2v, c2a, d2a, d2j, 
    c3p, d3p, d3v,
    a4p, c4p, d4p, b4v, d4v, c4a, d4j,
    a5p, b5p, c5p, d5p, b5v, c5v, d5v, c5a, d5a, d5j; 
  T endPos;
  std::mutex mtx;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * PathPlannerConstJerk instance to an output stream.\n
 * Does not print a newline control character.
 */
template<typename T>
std::ostream &operator<<(std::ostream &os, PathPlannerConstJerk<T> &pp) {
  os << "Block path planner constant jerk: '" << pp.getName() << "'";
  return os;
}

};
};

#endif /* ORG_EEROS_CONTROL_PATHPLANNERCONSTJERK_HPP_ */

