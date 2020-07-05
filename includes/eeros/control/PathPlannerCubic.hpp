#ifndef ORG_EEROS_CONTROL_PATHPLANNER_CUBIC_HPP_
#define ORG_EEROS_CONTROL_PATHPLANNER_CUBIC_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/System.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <mutex>
#include <atomic>

namespace eeros {
namespace control {
  
/**
 * This path planner takes precalculated cubic splines from a file and outputs the
 * resulting values for jerk, acceleration, velocity and position onto its outputs.
 * The file must contain piecewise information about the jerk within a given interval
 * together with the start conditions for acceleration, velocity and position at the 
 * beginning of the interval. You must make sure, that these initial values are the results
 * from the last interval.
 * The trajectory may be scaled in time and jerk in order to achieve a positional change
 * within a given time interval.
 * 
 * @since v1.0
 */

class PathPlannerCubic: public Block {
 public:
  /**
   * Constructs a path planner with a trajectory given in a predefined file.
   *
   * @param dt - sampling time
   */
  PathPlannerCubic(double dt) : posOut(this), velOut(this), accOut(this), jerkOut(this), dt(dt), interval(-dt) {
    prevPos = 0; prevVel = 0; prevAcc = 0;
  }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  PathPlannerCubic(const PathPlannerCubic& s) = delete; 

  /**
   * Choose a file which holds a trajectory. 
   * 
   * @param filename - name of the trajectory file
   */
  virtual void init(std::string filename) {
    std::ifstream file;       
    file.open(filename.c_str());   
    if (!file.is_open()) throw Fault("File for loading trajectory cannot be opened");
    clear();
    while(!file.eof()) {
      std::string tmp_string;
      double tmp_double;
      
      // time
      file >> tmp_string;
      tmp_double = ::atof(tmp_string.c_str());
      if (tmp_string.empty()) break;
      timeCoeffRaw.push_back(tmp_double);
      
      // jerk
      file >> tmp_string;
      tmp_double = ::atof(tmp_string.c_str());
      jerkCoeffRaw.push_back(tmp_double);
      
      // acc
      file >> tmp_string;
      tmp_double = ::atof(tmp_string.c_str());
      accCoeffRaw.push_back(tmp_double);
      
      // vel
      file >> tmp_string;
      tmp_double = ::atof(tmp_string.c_str());
      velCoeffRaw.push_back(tmp_double);
      
      // pos
      file >> tmp_string;
      tmp_double = ::atof(tmp_string.c_str());
      posCoeffRaw.push_back(tmp_double);
    }
  }
  
  /**
   * Runs the path planner block.
   */
  virtual void run() {
    double pos, vel, acc, jerk;
    
    std::lock_guard<std::mutex> guard(m);
    if (!finished && timeCoeff.size() > 0 && index < timeCoeff.size()) {
      if (t <= interval + (dt / 2)) {
        jerk = jerkCoeff[index];
        acc = prevAcc + jerk * dt;
        vel = prevVel + prevAcc * dt + jerk / 2 * dt * dt;
        pos = prevPos + prevVel * dt + prevAcc / 2 * dt * dt + jerk / 6 * dt * dt * dt;
      } else {
        if (first) {index = 0; interval = timeCoeff[index]; first = false;}
        else {
          index++;
          if (index == timeCoeff.size()) {
            finished = true;
            return;
          }
          interval += timeCoeff[index];
        }
        jerk = jerkCoeff[index];
        acc  = accCoeff[index];
        vel  = velCoeff[index];
        pos  = posCoeff[index];
      }
    } else {
      t = 0;
      index = 0;
      interval = -dt;
      first = true;
      pos = prevPos;
      vel = prevVel;
      acc = prevAcc;
      jerk = prevJerk;
    }
    t += dt;
    prevPos = pos;
    prevVel = vel;
    prevAcc = acc;
    prevJerk = jerk;
    
    posOut.getSignal().setValue(pos);
    velOut.getSignal().setValue(vel);
    accOut.getSignal().setValue(acc);
    jerkOut.getSignal().setValue(jerk);
    
    timestamp_t time = System::getTimeNs();
    posOut.getSignal().setTimestamp(time);
    velOut.getSignal().setTimestamp(time);
    accOut.getSignal().setTimestamp(time);
    jerkOut.getSignal().setTimestamp(time);
  }
    
  /**
   * Dispatches a new trajectory. The trajectory is taken from the path file and scaled 
   * so that it moves the distance given by deltaPos within the desired time. The position 
   * values are further shifted by startPos. 
   * 
   * @param time - total time for the trajectory to run
   * @param startPos - start position from where the trajectory will set off
   * @param deltaPos - distance the trajectory will cover
   * @return - the trajectury could be successfully started
   *
   * @see init(std::string filename)
   */
  virtual bool move(double time, double startPos, double deltaPos) {
    if (!finished) return false;
    if (timeCoeffRaw.size() <= 0) throw Fault("Path planner: time coeff array empty"); 
    
    scalePath(time, deltaPos); 
    
    for (std::size_t i = 0; i < posCoeff.size(); i++) posCoeff[i] += startPos;  // set start position
    finished = false;
    return true;
  }
  
  /**
   * Dispatches a new trajectory. The trajectory is taken from the path file, no scaling
   * is made. The trajectory starts from startPos and moves the distance given in the 
   * trajectory path file within the time given in the same file.
   * 
   * @param startPos - start position from where the trajectory will set off
   * @return - the trajectury could be successfully started
   *
   * @see init(std::string filename)
   */
  virtual bool move(double startPos) {
    if (!finished) return false;
    if (timeCoeffRaw.size() <= 0) throw Fault("Path planner: time coeff array empty"); 
    timeCoeff.resize(timeCoeffRaw.size());
    jerkCoeff.resize(timeCoeffRaw.size());
    accCoeff.resize(timeCoeffRaw.size());
    velCoeff.resize(timeCoeffRaw.size());
    posCoeff.resize(timeCoeffRaw.size());
    for (std::size_t i = 0; i < timeCoeffRaw.size(); i++) {
      timeCoeff[i] = timeCoeffRaw[i];
      jerkCoeff[i] = jerkCoeffRaw[i];
      accCoeff[i] = accCoeffRaw[i];
      velCoeff[i] = velCoeffRaw[i];
      posCoeff[i] = posCoeffRaw[i] + startPos;  // set start position
    }
    finished = false;
    return true;
  }
  
  /**
   * Query if a requested trajectory has already reached its end position.
   *
   * @return - end of trajectory is reached
   */
  virtual bool endReached() {return finished;}
  

  /**
   * Stop the current trajectory.
   */
  virtual void reset() {
    this->finished = true;
  }
  
  /**
   * Getter function for the position output.
   * 
   * @return The position output
   */
  virtual Output<>& getPosOut() {return posOut;}
  
  /**
   * Getter function for the velocity output.
   * 
   * @return The velocity output
   */
  virtual Output<>& getVelOut() {return velOut;}
  
  /**
   * Getter function for the acceleration output.
   * 
   * @return The acceleration output
   */
  virtual Output<>& getAccOut() {return accOut;}
  
  /**
   * Getter function for the jerk output.
   * 
   * @return The jerk output
   */
  virtual Output<>& getJerkOut() {return jerkOut;}
  
 private:
  void clear() {
    timeCoeffRaw.clear();
    jerkCoeffRaw.clear();
    accCoeffRaw.clear();
    velCoeffRaw.clear();
    posCoeffRaw.clear();
    timeCoeff.clear();
    jerkCoeff.clear();
    accCoeff.clear();
    velCoeff.clear();
    posCoeff.clear();
  }
    
  void scalePath(double time, double deltaPos) {
    std::vector<double> timeRounded, jerkRounded;
    
    // Get total time of curve
    double tabSize = timeCoeffRaw.size();
    double timeTotal = 0.0;
    for (int i = 0; i < tabSize; i++) timeTotal = timeTotal + timeCoeffRaw[i];
    
    // Scale time vector and round to multiple of sampling time
    double timeScale = time / timeTotal;
    for (int i = 0; i < tabSize; i++) 
      timeRounded.push_back(round(timeCoeffRaw[i] * timeScale / dt) * dt);
    
    // Get total time of rounded time spans
    double timeTotalRounded = 0.0;
    for (int i = 0; i < tabSize; i++) timeTotalRounded = timeTotalRounded + timeRounded[i];
    timeScale = timeTotalRounded / timeTotal;

    // Scale jerk according to scaled time and scaled position
    double posScale  = deltaPos / posCoeffRaw[tabSize-1];
    for(int i = 0; i < tabSize; i++) 
      jerkRounded.push_back(jerkCoeffRaw[i] * posScale / (timeScale*timeScale*timeScale));
      
    // Scale acc, vel and pos coefficients
    std::vector<double> accRounded, velRounded, posRounded;
    accRounded.resize(jerkRounded.size());
    velRounded.resize(jerkRounded.size());
    posRounded.resize(jerkRounded.size());
    double acc_prev = 0.0; accRounded[0] = acc_prev;
    double vel_prev = 0.0; velRounded[0] = vel_prev;
    double pos_prev = 0.0; posRounded[0] = pos_prev;
    for (int i = 1; i < tabSize; i++) {
      double t = timeRounded[i-1];
      double j = jerkRounded[i-1];
      accRounded[i] = acc_prev + j * t;
      velRounded[i] = vel_prev + acc_prev * t + j * t * t / 2;
      posRounded[i] = pos_prev + vel_prev * t + acc_prev * t * t / 2 + j * t * t * t / 6;
      acc_prev = accRounded[i];
      vel_prev = velRounded[i];
      pos_prev = posRounded[i];
    }
    
    std::lock_guard<std::mutex> guard(m);
    timeCoeff = timeRounded;
    jerkCoeff = jerkRounded;
    accCoeff = accRounded;
    velCoeff = velRounded;
    posCoeff = posRounded;
  }
  
  Output<> posOut, velOut, accOut, jerkOut; 
  bool finished = true;
  double dt, interval, t;
  std::size_t index = 0;
  bool first = true;
  double prevJerk, prevAcc, prevVel, prevPos;
  std::vector<double> timeCoeffRaw, jerkCoeffRaw, accCoeffRaw, velCoeffRaw, posCoeffRaw;
  std::vector<double> timeCoeff, jerkCoeff, accCoeff, velCoeff, posCoeff;
  std::mutex m;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * PathPlannerCubic instance to an output stream.\n
 * Does not print a newline control character.
 */
template<typename T>
std::ostream &operator<<(std::ostream &os, PathPlannerCubic &pp) {
  os << "Block path planner cubic: '" << pp.getName() << "'";
  return os;
}

};
};

#endif /* ORG_EEROS_CONTROL_PATHPLANNER_CUBIC_HPP_ */
