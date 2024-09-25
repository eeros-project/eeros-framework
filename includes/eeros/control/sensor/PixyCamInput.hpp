#ifndef ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_
#define ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_

#ifdef EEROS_USE_PIXYCAM

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/PixyCam.hpp>

using namespace eeros::hal;
using namespace eeros::math;
using namespace eeros::control;

namespace eeros {
namespace control {

/**
 * This block reads a Pixy2 camera over USB2.
 *
 * @since v1.3
 */

class PixyCamInput : public Blockio<0,1,Vector3> {
 public:
  /**
   * Constructs an input block to get data from a pixy camera.\n
   *
   * @see  BaumerOM70Input(std::string dev, int port, int slave, int priority)
   * @param dev - string with device name
   * @param priority - execution priority or PixyCam thread, to get sensors data
   */
  PixyCamInput(std::string dev, int priority = 5) : p(dev, priority) { }
 
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  PixyCamInput(const PixyCamInput& s) = delete; 

  /**
   * Gets the raw output
   * 
   * @return raw output
   */
  virtual Output<Vector3>& getOutRaw() {
    return outRaw;
  }

  /**
   * Gets the dot output
   * 
   * @return dot output
   */
  virtual Output<Matrix<nrDots,2,double>>& getOutDots() {
    return outDots;
  }

  /**
   * Gets the output height
   * 
   * @return output height
   */
  virtual Output<double>& getOutHeight() {
    return outHeight;
  }
  
  /**
   * Gets the state of the output
   * 
   * @return output is valid
   */
  virtual Output<bool>& getOutIsValid() {
    return outValid;
  }

  /**
   * Sets the state of the lamp
   * 
   * @param white state of the white lamp
   * @param rgb state of the rgb lamp
   */
  virtual void setLamp(bool white, bool rgb) {
    p.setLamp(white, rgb);
  }

  /**
   * Gets the number of detected blocks
   * 
   * @return number of detected blocks
   */
  virtual int getNofBlocks() {
    return p.getNofBlocks();
  }

  /**
   * Gets input data from the camera thread and outputs it
   */
  virtual void run() {
    out.getSignal().setValue(p.getPos());
    auto t = eeros::System::getTimeNs();
    out.getSignal().setTimestamp(t);
  
    outRaw.getSignal().setValue(p.getPosRaw());
    outRaw.getSignal().setTimestamp(t);
  
    outDots.getSignal().setValue(p.getDots());
    outDots.getSignal().setTimestamp(t);
  
    outHeight.getSignal().setValue(p.getHeight());
    outHeight.getSignal().setTimestamp(t);
  
    outValid.getSignal().setValue(p.isDataValid());
    outValid.getSignal().setTimestamp(t);
  }

 protected:
  PixyCam p;
  Output<Vector3> outRaw;
  Output<Matrix<nrDots,2,double>> outDots;
  Output<double> outHeight;
  Output<bool> outValid;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * sensor instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, PixyCamInput& sensor) {
  os << "Block PixyCAM input: '" << sensor.getName(); 
  return os;
}

}
}

#endif
#endif /* ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_ */

