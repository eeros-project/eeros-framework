#pragma once

#include <type_traits>
#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include "ros2/Ros2TypedefHelper.hpp"

namespace eeros {
namespace control {

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)

/**
 * This class is to convert a 3D vector to a 2D Vector (Alingend in a plane).
 * Input[i] and output[i] belongs together.
 * 
 * @tparam N Defines how many vectors has to be transformed.
 */
template<uint8_t N = 1>
class Ros2Vector3DTo2D : public control::Blockio<0,0> {
 public:
 
  /**
   * Special constructor if all vectors are in the same 2D plane 
   * 
   * @param provPlane: Specifies the plane to which the 2D vector is aligned.
   * @param swap: True if 2D vector has rotated in order (first y the x or first z then y) 
   */
  Ros2Vector3DTo2D(const Plane_t reqPlane, 
                   bool swap = false)
    : Ros2Vector3DTo2D(std::vector<Plane_t>(N, reqPlane), std::vector<bool>(N, swap)) {}
  
  /**
   * General constructor if all vectors are in different 2D planes 
   * 
   * @param provPlane: Specifies the plane to which the 2D vector is aligned.
   * @param swap: True if 2D vector has rotated in order (first y the x or first z then y) 
   */
  Ros2Vector3DTo2D(const std::vector<Plane_t> &reqPlane,
                   const std::vector<bool> &swap = false)
    : reqPlane_(reqPlane),
      swap_(swap) {
    if(reqPlane.size() != N){
      std::ostringstream oss;
      oss << __FILE__ << "::" << __LINE__ << "\n\t\t\t\t\t Provided Planes and swap must have the same size as N (plans size: " << reqPlane.size() << ",swap size: " << swap.size() << ", N size: " << std::to_string(N) << ")";
      throw eeros::Fault(oss.str());
    }
  }

  /* switching farbic */
  void run() override {
    for(int i = 0; i<N; i++){
      inTemp = this->in[i].getSignal().getValue();
      switch(reqPlane_[i]){
        case PLANE_XY:
          if(swap_[i]){
            outTemp.set(0,0, inTemp[1]);
            outTemp.set(1,0, inTemp[0]);
          } else {
            outTemp.set(0,0, inTemp[0]);
            outTemp.set(1,0, inTemp[1]);
          }
        break;
        case PLANE_XZ:
          if(swap_[i]){
            outTemp.set(0,0, inTemp[2]);
            outTemp.set(1,0, inTemp[0]);
          } else {
            outTemp.set(0,0, inTemp[0]);
            outTemp.set(1,0, inTemp[2]);
          }
        break;
        case PLANE_YZ:
          if(swap_[i]){
            outTemp.set(0,0, inTemp[2]);
            outTemp.set(1,0, inTemp[1]);
          } else {
            outTemp.set(0,0, inTemp[1]);
            outTemp.set(1,0, inTemp[2]);
          }
        break;
        default:
          std::ostringstream oss;
          oss << __FILE__ << "::" << __LINE__ << "\n\t\t\t\t\t In the switch of an enum, the default case is entred. Big Upsii";
          throw eeros::Fault(oss.str());
        break;
      }
      this->out[i].getSignal().setValue(outTemp);
      this->out[i].getSignal().setTimestamp(this->in[i].getSignal().getTimestamp());
    }
  }

  /**
   * Get an input of the block.
   * 
   * @param index - index of the input
   * @return input
   */
  virtual Input<math::Vector3>& getIn(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return in[index];
  }

  /**
   * Get the output of the block.
   * 
   * @return output
   */
  virtual Output<math::Vector2>& getOut(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return out[index];
  }

 private:
  const std::vector<Plane_t> reqPlane_;
  const std::vector<bool> swap_;
  math::Vector3 inTemp;
  math::Vector2 outTemp;

  Input<math::Vector3> in[N];
  Output<math::Vector2> out[N];
};

} /* END Namespace: control */
} /* END Namespace: eeros */