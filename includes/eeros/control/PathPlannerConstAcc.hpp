#ifndef ORG_EEROS_CONTROL_PATHPLANNERCONSTACC_HPP_
#define ORG_EEROS_CONTROL_PATHPLANNERCONSTACC_HPP_

#include <eeros/control/Output.hpp>
#include <eeros/control/TrajectoryGenerator.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/System.hpp>
#include <cmath>
#include <mutex>

using namespace eeros::logger;

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
   * (will not be higher than velmax). For the third part the trajectory will stop with a 
   * constant deceleration (will not be higher decMax).
   * The sampling time must be set to the time with which the timedomain containing this block will run.
   *
   * @param velMax - maximum velocity
   * @param accMax - maximum acceleration
   * @param decMax - maximum deceleration
   * @param dt - sampling time
   */
  PathPlannerConstAcc(T velMax, T accMax, T decMax, double dt) 
      : finished(true), velMax(velMax), accMax(accMax), decMax(decMax), dt(dt) { }
  
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
        if (t >= 0 && t < dT1) {
          y[0][i] = a1p[i] * pow(t, 2) + c1p[i];
          y[1][i] = b1v[i] * t;
          y[2][i] = c1a[i];
        } else if (t >= dT1 && t < dT1 + dT2) {
          y[0][i] = b2p[i] * t + c2p[i];
          y[1][i] = c2v[i];
          y[2][i] = 0;
        } else if (t >= dT1 + dT2 && t <= dT1 + dT2 + dT3) {
          y[0][i] = a3p[i] * pow(t, 2) + b3p[i] * t + c3p[i];
          y[1][i] = b3v[i] * t + c3v[i];
          y[2][i] = c3a[i];
        } else if(t > dT1 + dT2 + dT3) {
          finished = true;
          y[0][i] = this->last[0][i];
          y[1][i] = 0.0;
          y[2][i] = 0.0;
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
    
    T zero; zero = 0;
    if (distance == zero) return false;
    
    // Define speeds and accelerations
    for (unsigned int i = 0; i < calcVelNorm.size(); i++) {
      calcVelNorm[i] = fabs(velMax[i] / distance[i]);
      calcAccNorm[i] = fabs(accMax[i] / distance[i]);
      calcDecNorm[i] = fabs(decMax[i] / distance[i]);
    }
    
    // Init velNorm, accNorm, decNorm and find minimum
    velNorm = calcVelNorm[0]; accNorm = calcAccNorm[0]; decNorm = calcDecNorm[0]; 
    for (unsigned int i = 0; i < calcVelNorm.size(); i++) {
      if (calcVelNorm[i] < velNorm) velNorm = calcVelNorm[i];
      if (calcAccNorm[i] < accNorm) accNorm = calcAccNorm[i];
      if (calcDecNorm[i] < decNorm) decNorm = calcDecNorm[i];
    }
    
    // Minimize velocity
    E squareNormVel = sqrt(2 * (accNorm * decNorm) / (accNorm + decNorm));
    if (velNorm > squareNormVel) velNorm = squareNormVel; 
    
    // Calculate time intervals    
    dT1 = velNorm / accNorm;
    dT3 = velNorm / decNorm;
    dT2 = 1 / velNorm - (dT1 + dT3) * 0.5;
    if (dT2 < 0) dT2 = 0;
    
    // Adaptation to timestamps
    dT1 = ceil(dT1 / dt) * dt;
    dT2 = ceil(dT2 / dt) * dt;
    dT3 = ceil(dT3 / dt) * dt; 
    
    // Adaptation of speed to new timestamps
    velNorm = 1/((dT2 + (dT1 + dT3) * 0.5) *dt);
    
    a1p = 0.5 * velNorm / dT1 * distance * dt;
    c1p = start[0];
    b1v = velNorm / dT1 * dt * distance * dt;
    c1a = velNorm * dt / dT1 * distance * pow(dt, 2);
    
    b2p = velNorm * dt * distance;
    c2p = start[0] - 0.5 * velNorm * dT1 * dt * distance;
    c2v = velNorm * distance * dt * dt;
    c2a = 0; 
    
    a3p = (-1) * velNorm * 0.5 * dt / dT3 * distance;
    b3p = velNorm * dt / dT3 * (dT1 + dT2 + dT3) * distance;
    c3p = start[0] + (1 - velNorm * 0.5 * dt / dT3 * pow(dT1 + dT2 + dT3, 2)) * distance;
    b3v = (-1) *velNorm * dt * distance / dT3 * dt;  
    c3v = velNorm * (dT1 + dT2 + dT3) * distance * dt / dT3 * dt; 
    c3a = (-1) * velNorm * dt / dT3 * distance * dt * dt;
    
    std::lock_guard<std::mutex> lck(mtx);
    finished = false;
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
   * Sets the maximum value for the acceleration. The maximum acceleration is used for the start of the trajectory.
   *
   * @param acc - maximum acceleration
   */
  virtual void setMaxAcc(T acc) {accMax = acc;}
  
  /**
   * Sets the maximum value for the deceleration. The maximum deceleration is used for the end of the trajectory.
   *
   * @param dec - maximum deceleration
   */
  virtual void setMaxDec(T dec) {decMax = dec;}
  
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
  T velMax, accMax, decMax;
  double t, dt, dT1, dT2, dT3;
  T a1p, c1p, b1v, c1a, b2p, c2p, c2v, c2a, a3p, b3p, c3p, b3v, c3v, c3a; 
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

