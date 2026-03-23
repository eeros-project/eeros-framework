#ifndef ORG_EEROS_CONTROL_MOUSEINPUT_HPP_
#define ORG_EEROS_CONTROL_MOUSEINPUT_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/Mouse.hpp>

using namespace eeros::math;
using namespace eeros::hal;

namespace eeros {
namespace control {

/**
 * This block serves to read the input from a standard mouse with x, y coordinates, two
 * wheels and three buttons. The range of all the input parameters together with the scaling 
 * can be choosen as needed.
 *
 * @since v0.6
 */
class MouseInput: public Blockio<0,1,Vector4> {
 public:
  MouseInput(std::string dev, int priority = 20);
  MouseInput(std::string dev, Vector4 scale, Vector4 min, Vector4 max, int priority = 20);

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  MouseInput(const MouseInput& s) = delete; 

  Output<Matrix<3,1,bool>>& getButtonOut();
  void run() override;
  virtual void setInitPos(double x, double y, double z, double r);
  virtual void setInitPos(Matrix<4> pos);
  virtual void reset(double x, double y, double z, double r);

protected:
  Mouse mouse;
  Output<Matrix<3,1,bool>> buttonOut;
  double x, y, z, r;
  bool first;
  double axisScale_x = 0.0001;
  double axisScale_y = 0.0001;
  double axisScale_z = 0.001;
  double axisScale_r = 0.2;
  double min_x = -0.03;
  double max_x = 0.045;
  double min_y = -0.03;
  double max_y = 0.03;
  double min_z = -0.063;
  double max_z = -0.015;
  double min_r = -2.8;
  double max_r = 2.8;

};

};
};

#endif /* ORG_EEROS_CONTROL_MOUSEINPUT_HPP_ */
