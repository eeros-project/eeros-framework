#pragma once

#include <type_traits>
#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include "ros2/Ros2TypedefHelper.hpp"

namespace eeros {
namespace control {

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)

/**
 * This class is to convert a 3D Vector to a double (Alingend in a axe).
 * Input[i] and output[i] belongs together.
 * 
 * @tparam N Defines how many vectors has to be transformed.
 */
template<uint8_t N = 1>
class Ros2Vector3DToDouble : public control::Blockio<0,0> {
 public:
 
  /**
   * Special constructor if all doubles are in the same axe
   * and the other value has to be filled with the same value by all.
   * 
   * @param reqAxe_: Specifies the axe to which the value is aligned.
   * @param valueToFill: Defines the value with which the third dimension must be filled.
   */
  Ros2Vector3DToDouble(Axe_t reqAxe)
    : Ros2Vector3DToDouble(std::vector<Axe_t>(N, reqAxe)) {}
    
  /**
   * General constructor if all doubles are in different 2D planes 
   * and the third value has to be different
   * 
   * @param reqAxe_: Specifies the axe to which the value is aligned.
   * @param valueToFill: Defines the value with which the other dimension must be filled.
   */
  Ros2Vector3DToDouble(const std::vector<Axe_t> &reqAxe)
    : reqAxe_(reqAxe) {
    if(reqAxe_.size() != N){
      std::ostringstream oss;
      oss << __FILE__ << "::" << __LINE__ << "\n\t\t\t\t\t provided Axe must have the same size as N (axe size: " << reqAxe_.size() << ", N size: " << std::to_string(N) << ")";
      throw eeros::Fault(oss.str());
    }
  }

  /* switching farbic */
  void run() override {
    for(int i = 0; i<N; i++){
      inTemp = this->in[i].getSignal().getValue();
      switch(reqAxe_[i]){
        case AXE_X:
          outTemp = inTemp[0];
        break;
        case AXE_Y:
          outTemp = inTemp[1];
        break;
        case AXE_Z:
          outTemp = inTemp[2];
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
  virtual Output<double>& getOut(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return out[index];
  }

 private:
  const std::vector<Axe_t> reqAxe_;
  double outTemp;
  math::Vector3 inTemp;

  Input<math::Vector3> in[N];
  Output<double> out[N];
};

} /* END Namespace: control */
} /* END Namespace: eeros */