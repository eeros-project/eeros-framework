#pragma once

#include <type_traits>
#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/ros2/Ros2TypedefHelper.hpp>

namespace eeros {
namespace control {

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)

/**
 * This class is to convert a double (Alingend in a axe) to a 3D Vector.
 * Input[i] and output[i] belongs together.
 * 
 * @tparam N Defines how many vectors has to be switched.
 */
template<uint8_t N = 1>
class Ros2DoubleTo3DVector : public control::Blockio<0,0> {
 public:
 
  /**
   * Special constructor if all doubles are in the same axe
   * and the other value has to be filled with the same value by all.
   * 
   * @param provPlane: Specifies the axe to which the value is aligned.
   * @param valueToFill: Defines the value with which the third dimension must be filled.
   */
  Ros2DoubleTo3DVector(Axe_t provAxe, 
                       math::Vector2 valueToFill)
    : Ros2DoubleTo3DVector(std::vector<Axe_t>(N, provAxe), std::vector<math::Vector2>(N, valueToFill)) {}
  
  /**
   * Special constructor if all doubles are in different axes 
   * and the other values has to be filled with the same value by all.
   * 
   * @param provPlane: Specifies the axe to which the value is aligned.
   * @param valueToFill: Defines the value with which the other dimension must be filled.
   */
  Ros2DoubleTo3DVector(const std::vector<Axe_t> &provAxe, 
                       math::Vector2 valueToFill)
    : Ros2DoubleTo3DVector(provAxe, std::vector<math::Vector2>(N, valueToFill)) {}
  
  /**
   * General constructor if all doubles are in different 2D planes 
   * and the third value has to be different
   * 
   * @param provPlane: Specifies the axe to which the value is aligned.
   * @param valueToFill: Defines the value with which the other dimension must be filled.
   */
  Ros2DoubleTo3DVector(const std::vector<Axe_t> &provAxe,
                       const std::vector<math::Vector2> &valueToFill)
    : provAxe_(provAxe),
      valueToFill_(valueToFill) {
    if(provAxe.size() != N || valueToFill.size() != N){
      std::ostringstream oss;
      oss << __FILE__ << "::" << __LINE__ << "\n\t\t\t\t\t provided Axe and valueToFill must have the same size as N (Axe size: " << provAxe_.size() << ",valueToFill size: " << valueToFill.size() << ", N size: " << std::to_string(N) << ")";
      throw eeros::Fault(oss.str());
    }
  }

  /* switching fabric */
  void run() override {
    for(int i = 0; i<N; i++){
      inTemp = this->in[i].getSignal().getValue();
      switch(provAxe_[i]){
        case AXE_X:
            outTemp.set(0,0, inTemp);
            outTemp.set(1,0, valueToFill_[i][0]);
            outTemp.set(2,0, valueToFill_[i][1]);
        break;
        case AXE_Y:
            outTemp.set(0,0, valueToFill_[i][0]);
            outTemp.set(1,0, inTemp);
            outTemp.set(2,0, valueToFill_[i][1]);
        break;
        case AXE_Z:
            outTemp.set(0,0, valueToFill_[i][0]);
            outTemp.set(1,0, valueToFill_[i][1]);
            outTemp.set(2,0, inTemp);
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
  virtual Input<double>& getIn(uint8_t index) {
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
  const std::vector<Axe_t> provAxe_;
  std::vector<math::Vector2> valueToFill_;
  double inTemp;
  math::Vector3 outTemp;

  Input<double> in[N];
  Output<math::Vector3> out[N];
};

} /* END Namespace: control */
} /* END Namespace: eeros */