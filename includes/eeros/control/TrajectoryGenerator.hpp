#ifndef ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_
#define ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_

#include <eeros/control/Block.hpp>
#include <array>

namespace eeros {
namespace control {

/**
 * A trajectory generator block is serves as template to build blocks which 
 * generate various trajectories, e.g trajectories which lead from a start to 
 * an end position with smooth velocity or smooth acceleration. 
 * 
 * @tparam T - output type (must be a composite type), 
 *             a trajectory in 3-dimensional space needs T = Matrix<3,1,double>, 
 *             a trajectory in linear space needs T = Matrix<1,1,double>
 * @tparam N - number of position and its derivatives, 
 *             e.g. a trajectory with N = 3 will include position, velocity and acceleration
 *             e.g. a trajectory with N = 4 will further add jerk
 *
 * @since v1.0
 */

template<typename T, int N = 1>
class TrajectoryGenerator : public Block {
 public:
  /**
   * Constructs a default instance and sets the initial values to 0.
   */
  TrajectoryGenerator() {
    for(auto& e : last) e = 0;
  }
  
  /**
   * Query if a requested trajectory has already reached its end position.
   *
   * @return - end of trajectory is reached
   */
  virtual bool endReached() = 0;
  
  /**
   * Dispatches a new trajectory from start to end.
   * Both parameters can be generally described with arrays containing the start position
   * together with its higher derivatives, e.g. start velocity, start acceleration etc. and 
   * the end position together with the higher derivatives, e.g. end velocity, end acceleration etc.
   * 
   * @param start - array containing start position and its higher derivatives
   * @param end - array containing end position and its higher derivatives
   * @return - a trajectory could be sucessfully generated for this parameters
   *
   * @see setStart()
   * @see run()
   */
  virtual bool move(std::array<T, N> start, std::array<T, N> end) = 0;
  
  /**
   * Dispatches a new trajectory from the current position to the end position.
   * The higher derivatives of the position are set to 0.
   * 
   * @param end - end position
   * @return - a trajectory could be sucessfully generated for this parameters
   *
   * @see setStart()
   * @see run()
   */
  virtual bool move(T end) {
    std::array<T, N> e;
    for(auto& i : e) i = 0;
    e[0] = end;
    return move(last, e);
  }
  
  /**
   * Dispatches a new trajectory from the current position and state to end position and state.
   * 
   * @param end - array containing end position and its higher derivatives
   * @return - a trajectory could be sucessfully generated for this parameters
   *
   * @see setStart()
   * @see run()
   */
  virtual bool move(std::array<T, N> end) {
    return move(last, end);
  }
  
  /**
   * Dispatches a new trajectory from the start position to the end position.
   * The higher derivatives of the position are set to 0.
   * 
   * @param start - start position
   * @param end - end position
   * @return - a trajectory could be sucessfully generated for this parameters
   *
   * @see setStart()
   * @see run()
   */
  virtual bool move(T start, T end) {
    std::array<T, N> s, e;
    for(auto& i : e) i = 0; 
    for(auto& i : s) i = 0;
    s[0] = start; e[0] = end;
    return move(s, e);
  }
  
  /**
   * Sets the start position and state of the higher derivatives (velocity, acceleration ...)
   * 
   * @param start - array containing start position and its higher derivatives
   */
  virtual void setStart(std::array<T, N> start) = 0;
  
  /**
   * Sets the start position. The higher derivatives of the position are set to 0.
   * 
   * @param start - start position
   */
  virtual void setStart(T start) {
    std::array<T, N> e;
    for(auto& i : e) i = 0;
    e[0] = start;
    setStart(e);
  }
  
 protected:
  std::array<T, N> last;
};

};
};

#endif /* ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_ */

