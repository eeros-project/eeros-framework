#ifndef ORG_EEROS_CONTROL_MOUSEINPUT_HPP_
#define ORG_EEROS_CONTROL_MOUSEINPUT_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/Mouse.hpp>
#include <string>

namespace eeros::control {

/**
* @brief Block for reading input from a standard USB mouse.
 *
 * Reads x/y translation, scroll wheel (z), and rotation (r) axes from a
 * standard mouse device, along with three button states. All axes are scaled
 * and clamped to configurable ranges before being published on the output
 * signal.
 *
 * The block exposes two outputs:
 * - A @c Vector4 output (inherited from @c Blockio) carrying [x, y, z, r]
 * - A @c Matrix<3,1,bool> output carrying the three button states
 *
 * @par Example
 * @code
 * MouseInput mouse("/dev/input/mice");
 * mouse.setInitPos(0.0, 0.0, -0.04, 0.0);
 * @endcode
 *
 * @since v0.6
 */
class MouseInput: public Blockio<0,1,math::Vector4> {
 public:
  /**
   * @brief Constructs a MouseInput block with default scaling and range.
   *
   * @param dev      path to the mouse device (e.g. @c "/dev/input/mice")
   * @param priority thread priority for the underlying mouse HAL thread
   */
  MouseInput(std::string dev, int priority = 20);
  MouseInput(std::string dev, eeros::math::Vector4 scale, eeros::math::Vector4 min, eeros::math::Vector4 max, int priority = 20);
  
  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  MouseInput(const MouseInput& s) = delete;
  MouseInput& operator=(const MouseInput&) = delete;

  Output<eeros::math::Matrix<3,1,bool>>& getButtonOut();
  virtual void run();
  virtual void setInitPos(double x, double y, double z, double r);
  virtual void setInitPos(math::Vector4 pos);
  virtual void reset(double x, double y, double z, double r);

protected:
  hal::Mouse mouse;
  Output<eeros::math::Matrix<3,1,bool>> buttonOut;
  double x, y, z, r;
  bool first{true};
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

}

#endif /* ORG_EEROS_CONTROL_MOUSEINPUT_HPP_ */
