#ifndef ORG_EEROS_CONTROL_MOUSEINPUT_HPP_
#define ORG_EEROS_CONTROL_MOUSEINPUT_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/Mouse.hpp>
#include <string>


namespace eeros::control {

using namespace eeros::math;
using namespace eeros::hal;

/**
 * @brief Block for reading input from a standard USB mouse.
 *
 * Reads x/y translation, scroll wheel (z), and rotation (r) axes from a
 * standard mouse device, along with three button states. All axes are scaled
 * and clamped to configurable ranges before being published on the output
 * signal.
 * While the output can carry SI units, the button state output cannot.
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
 * @tparam U - output signal unit type (dimensionless - default type)
 *
 * @since v0.6
 */
template < SIUnit U = SIUnit::create() >
class MouseInput: public Blockio<0,1,Vector4,Vector4,siunit::generateNSizeArray<0>(),MakeUnitArray<U>::value> {
 public:
  /**
   * @brief Constructs a MouseInput block with default scaling and range.
   *
   * @param dev      path to the mouse device (e.g. @c "/dev/input/mice")
   * @param priority thread priority for the underlying mouse HAL thread
   */
  MouseInput(std::string dev, int priority = 20) : mouse(dev, priority), buttonOut(this) {
    setInitPos(0, 0, 0, 0);
  }

  /**
   * @brief Constructs a MouseInput block with custom scaling and range.
   *
   * @param dev      path to the mouse device (e.g. @c "/dev/input/mice")
   * @param scale    per-axis scale factors [x, y, z, r]
   * @param min      per-axis minimum clamp values [x, y, z, r]
   * @param max      per-axis maximum clamp values [x, y, z, r]
   * @param priority thread priority for the underlying mouse HAL thread
   */
  MouseInput(std::string dev, Vector4 scale, Vector4 min, Vector4 max, int priority = 20)
      : mouse(dev, priority), buttonOut(this) {
    axisScale_x = scale(0);
    axisScale_y = scale(1);
    axisScale_z = scale(2);
    axisScale_r = scale(3);
    min_x = min(0);
    max_x = max(0);
    min_y = min(1);
    max_y = max(1);
    min_z = min(2);
    max_z = max(2);
    min_r = min(3);
    max_r = max(3);
    setInitPos(0, 0, 0, 0);
    first = true;
  }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  MouseInput(const MouseInput& s) = delete; 
  MouseInput& operator=(const MouseInput&) = delete;

  /**
   * @brief Returns the button output signal.
   *
   * The output carries a @c Matrix<3,1,bool> where each element corresponds
   * to one mouse button: [left, right, middle].
   *
   * @return reference to the button output
   */
  // Output<Matrix<3,1,bool>>& getButtonOut() {return buttonOut;}
  auto& getButtonOut() {return buttonOut;}

  /**
   * @brief Reads the mouse device and updates all output signals.
   *
   * Called periodically by the control system executor. Applies axis scaling
   * and clamping before writing to the output signals.
   */
  void run() override {
    auto current = mouse.current.read();
    if (first) {
      x = axisScale_x * -current.axis.y;
      y = axisScale_y * -current.axis.x;
      z = axisScale_z * -current.axis.z;
      r = axisScale_r * -current.axis.r;
      first = false;
    }

    double vx = axisScale_x * current.axis.x;
    if ((x + vx) < min_x) x = (min_x - vx);
    else if ((x + vx) > max_x) x = (max_x - vx);
    vx += x;

    double vy = axisScale_y * current.axis.y;
    if ((y + vy) < min_y) y = (min_y - vy);
    else if ((y + vy) > max_y) y = (max_y - vy);
    vy += y;

    double vz = axisScale_z * current.axis.z;
    if ((z + vz) < min_z) z = (min_z - vz);
    else if ((z + vz) > max_z) z = (max_z - vz);
    vz += z;

    double vr = axisScale_r * current.axis.r;
    if ((r + vr) < min_r) r = (min_r - vr);
    else if ((r + vr) > max_r) r = (max_r - vr);
    vr += r;

    uint64_t time = eeros::System::getTimeNs();
    this->out.getSignal().setValue(Vector4{ vx, vy, vz, vr });
    this->out.getSignal().setTimestamp(time);

    buttonOut.getSignal().setValue(Matrix<3,1,bool>{current.button.left, current.button.middle, current.button.right});
    buttonOut.getSignal().setTimestamp(time);
  }

  /**
   * @brief Sets the initial position of all axes.
   *
   * @param x  initial x position
   * @param y  initial y position
   * @param z  initial z position (scroll)
   * @param r  initial rotation
   */
  virtual void setInitPos(double x, double y, double z, double r) {
    reset(x, y, z, r);
    this->out.getSignal().setValue(Matrix<4>{ x, y, z, r });
  }

  /**
   * @brief Sets the initial position from a 4-element matrix.
   *
   * @param pos  initial position vector [x, y, z, r]
   */
  virtual void setInitPos(Matrix<4> pos) {
    setInitPos(pos[0], pos[1], pos[2], pos[3]);
  }

  /**
   * @brief Resets all axes to the given position.
   *
   * Unlike @ref setInitPos(), this can be called at any time during operation
   * to snap the current position to new values.
   *
   * @param x  reset x position
   * @param y  reset y position
   * @param z  reset z position (scroll)
   * @param r  reset rotation
   */
  virtual void reset(double x, double y, double z, double r) {
    auto current = mouse.current.read();
    this->x = (x - axisScale_x * current.axis.y);
    this->y = (y - axisScale_y * current.axis.x);
    this->z = (z - axisScale_z * current.axis.z);
    this->r = (r - axisScale_r * current.axis.r);
  }

 protected:
  Mouse mouse;
  Output<Matrix<3,1,bool>> buttonOut;
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
