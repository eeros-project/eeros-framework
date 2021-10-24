#ifndef ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_
#define ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_

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
 * This block reads a Pixy2 camera.
 *
 * @since v1.3
 */

class PixyCamInput : public Blockio<0,1,Vector3> {
public:
  PixyCamInput(std::string dev, int priority = 5) : p(dev, priority) { }
  
  virtual Output<Vector3>& getOutRaw() {
    return outRaw;
  }
  virtual Output<Matrix<nrDots,2,double>>& getOutDots() {
    return outDots;
  }
  virtual Output<double>& getOutHeight() {
    return outHeight;
  }
  virtual Output<bool>& getOutIsValid() {
    return outValid;
  }

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

  virtual void setLamp(bool white, bool rgb) {
    p.setLamp(white, rgb);
  }

  virtual int getNofBlocks() {
    return p.getNofBlocks();
  }

 protected:
  PixyCam p;
  Output<Vector3> outRaw;
  Output<Matrix<nrDots,2,double>> outDots;
  Output<double> outHeight;
  Output<bool> outValid;
};

}
}

#endif /* ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_ */

