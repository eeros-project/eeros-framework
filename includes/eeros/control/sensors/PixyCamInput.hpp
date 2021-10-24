#ifndef ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_
#define ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/PixyCam.hpp>


namespace eeros {
namespace control {

class PixyCamInput : public eeros::control::Block {
 public:
  PixyCamInput(std::string dev, int priority = 5) : p(dev, priority) {
  }
  
  virtual eeros::control::Output<eeros::math::Vector3>& getOut(){
	  return out;
  }
  virtual eeros::control::Output<eeros::math::Vector3>& getOut_raw(){
	  return out_raw;
  }
  virtual eeros::control::Output<eeros::math::Matrix<nr_dots,2,double>>& getOut_dots(){
	  return out_dots;
  }
  virtual eeros::control::Output<double>& getOut_height(){
	  return out_height;
  }
  virtual eeros::control::Output<bool>& getOut_isValid(){
	  return out_valid;
  }

  virtual void run() {
    out.getSignal().setValue(p.getPos());
    out.getSignal().setTimestamp(eeros::System::getTimeNs());
	
    out_raw.getSignal().setValue(p.getPos_raw());
    out_raw.getSignal().setTimestamp(eeros::System::getTimeNs());
	
    out_dots.getSignal().setValue(p.getDots());
    out_dots.getSignal().setTimestamp(eeros::System::getTimeNs());
	
    out_height.getSignal().setValue(p.getHeight());
    out_height.getSignal().setTimestamp(eeros::System::getTimeNs());
	
    out_valid.getSignal().setValue(p.isDataValid());
    out_valid.getSignal().setTimestamp(eeros::System::getTimeNs());
  }

  virtual void setLamp(bool white, bool rgb) {
    p.setLamp(white, rgb);;
  }

  virtual int getNofBlocks() {
    return p.getNofBlocks();
  }

 protected:
  eeros::hal::PixyCam p;
  eeros::control::Output<eeros::math::Vector3> out;
  eeros::control::Output<eeros::math::Vector3> out_raw;
  eeros::control::Output<eeros::math::Matrix<nr_dots,2,double>> out_dots;
  eeros::control::Output<double> out_height;
  eeros::control::Output<bool> out_valid;
};

}
}

#endif /* ORG_EEROS_CONTROL_PIXYCAM_INPUT_HPP_ */
