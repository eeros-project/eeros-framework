#pragma once

#include <type_traits>
#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

#include "ros2/Ros2TypedefHelper.hpp"


namespace eeros {
namespace control {

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)

/**
 * This class is to convert a 2D vector (Alingend in a plane) to a 3D Vector.
 * Input[i] and output[i] belongs together.
 * 
 * @tparam N Defines how many vectors has to be transformed.
 */
template<uint8_t N = 1>
class Ros2Vector2DTo3D : public control::Blockio<0,0> {
  static_assert(N != 0, "Template parameter N must not be zero");
 public:

  /**
   * Special constructor if all vectors are in the same 2D plane 
   * and the third value has to be filled with the same value by all.
   * 
   * @param provPlane: Specifies the plane to which the 2D vector is aligned.
   * @param valueToFill: Defines the value with which the third dimension must be filled.
   * @param swap: True if 2D vector has rotated in order (first y the x or first z then y) 
   */
  Ros2Vector2DTo3D(const Plane_t provPlane, 
                   double valueToFill = 0,
                   bool swap = false)
    : Ros2Vector2DTo3D(std::vector<Plane_t>(N, provPlane), std::vector<double>(N, valueToFill), swap) {}
  
  /**
   * Special constructor if all vectors are in different 2D planes 
   * and the third value has to be filled with the same value by all.
   * 
   * @param provPlane: Specifies the plane to which the 2D vector is aligned.
   * @param valueToFill: Defines the value with which the third dimension must be filled.
   * @param swap: True if 2D vector has rotated in order (first y the x or first z then y) 
   */
  Ros2Vector2DTo3D(const std::vector<Plane_t> provPlane, 
                   double valueToFill = 0,
                   bool swap = false)
    : Ros2Vector2DTo3D(provPlane, std::vector<double>(N, valueToFill), swap) {}
  
  /**
   * General constructor if all vectors are in different 2D planes 
   * and the third value has to be different
   * 
   * @param provPlane: Specifies the plane to which the 2D vector is aligned.
   * @param valueToFill: Defines the value with which the third dimension must be filled.
   * @param swap: True if 2D vector has rotated in order (first y the x or first z then y) 
   */
  Ros2Vector2DTo3D(const std::vector<Plane_t> &provPlane,
                   const std::vector<double> &valueToFill,
                   bool swap = false)
    : provPlane_(provPlane),
      valueToFill_(valueToFill),
      swap_(swap) {
    if(provPlane.size() != N || valueToFill.size() != N){
      std::ostringstream oss;
      oss << __FILE__ << "::" << __LINE__ << "\n\t\t\t\t\t Provided Planes and valueToFill must have the same size as N (plans size: " << provPlane.size() << ",valueToFill size: " << valueToFill.size() << ", N size: " << std::to_string(N) << ")";
      throw eeros::Fault(oss.str());
    }
  }

  /* switching farbic */
  void run() override {
    for(int i = 0; i<N; i++){
      inTemp = this->in[i].getSignal().getValue();
      switch(provPlane_[i]){
        case PLANE_XY:
          if(swap_){
            outTemp.set(0,0, inTemp[1]);
            outTemp.set(1,0, inTemp[0]);
            outTemp.set(2,0, valueToFill_[i]);
          } else {
            outTemp.set(0,0, inTemp[0]);
            outTemp.set(1,0, inTemp[1]);
            outTemp.set(2,0, valueToFill_[i]);
          }
        break;
        case PLANE_XZ:
          if(swap_){
            outTemp.set(0,0, inTemp[1]);
            outTemp.set(1,0, valueToFill_[i]);
            outTemp.set(2,0, inTemp[0]);
          } else {
            outTemp.set(0,0, inTemp[0]);
            outTemp.set(1,0, valueToFill_[i]);
            outTemp.set(2,0, inTemp[1]);
          }
        break;
        case PLANE_YZ:
          if(swap_){
            outTemp.set(0,0, valueToFill_[i]);
            outTemp.set(1,0, inTemp[1]);
            outTemp.set(2,0, inTemp[0]);
          } else {
            outTemp.set(0,0, valueToFill_[i]);
            outTemp.set(1,0, inTemp[0]);
            outTemp.set(2,0, inTemp[1]);
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
  virtual Input<math::Vector2>& getIn(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return in[index];
  }

  /**
   * Get the output of the block.
   * 
   * @return output
   */
  virtual Output<math::Vector3>& getOut(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return out[index];
  }

 private:
  const std::vector<Plane_t> provPlane_;
  const std::vector<double> valueToFill_;
  const bool swap_;
  math::Vector2 inTemp;
  math::Vector3 outTemp;

  Input<math::Vector2> in[N];
  Output<math::Vector3> out[N];
};

} /* END Namespace: control */
} /* END Namespace: eeros */