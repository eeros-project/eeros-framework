#ifndef ORG_EEROS_CONTROL_XBOXINPUT_HPP_
#define ORG_EEROS_CONTROL_XBOXINPUT_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/core/System.hpp>
#include <eeros/hal/XBox.hpp>
#include <eeros/math/Matrix.hpp>
#include <string>
#include <functional>

namespace eeros::control {

/**
 * @brief Block for reading input from an Xbox controller.
 *
 * Reads all analog axes and digital buttons from a standard Xbox-compatible
 * gamepad. Axis values are scaled and clamped to configurable ranges before
 * being published on the output signal.
 *
 * Two outputs are provided:
 * - A @c Matrix<XBOX_AXIS_COUNT> output (inherited from @c Blockio) carrying
 *   the scaled axis values
 * - A @c Matrix<XBOX_BUTTON_COUNT,1,bool> output carrying all button states,
 *   accessible via @ref getButtonOut()
 *
 * A callback can be registered via @ref on_button() to react to button
 * events outside the control loop.
 *
 * @par Example
 * @code
 * XBoxInput xbox("/dev/input/js0");
 * xbox.setInitPos(initPos);
 * xbox.on_button([](int btn, bool pressed) {
 *   if (btn == 0 && pressed) startMotion();
 * });
 * @endcode
 *
 * @since v1.0
 */
class XBoxInput: public Blockio<0,1,eeros::math::Matrix<XBOX_AXIS_COUNT>> {
 public:
  /**
   * @brief Constructs an XBoxInput block.
   *
   * Opens the controller device and starts the HAL reader thread.
   *
   * @param dev       Path to the joystick device (e.g. @c "/dev/input/js0")
   * @param priority  Thread priority for the underlying HAL reader thread
   */
  XBoxInput(std::string dev, int priority = 20);

  /**
   * @brief Destructs the block and releases the controller device.
   */
  virtual ~XBoxInput();

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  XBoxInput(const XBoxInput& s) = delete; 
  XBoxInput& operator=(const XBoxInput&) = delete;

  /**
   * @brief Reads the controller and updates all output signals.
   *
   * Called periodically by the control system executor. Applies per-axis
   * scaling and clamping before writing to the output signals.
   */
  void run() override;

  /**
   * @brief Sets the initial axis position.
   *
   * @param initPos  Initial position for all axes
   */
  void setInitPos(eeros::math::Matrix<XBOX_AXIS_COUNT> initPos);

  /**
   * @brief Sets a global speed scale factor applied to all axes.
   *
   * Multiplied with the per-axis scale factors on each @ref run() call.
   *
   * @param speedScale  Scale factor (default: 1.0)
   */
  void setSpeedScaleFactor(double speedScale);

  /**
   * @brief Returns the button state output signal.
   *
   * Each element corresponds to one controller button (true = pressed).
   *
   * @return Reference to the button output
   */
  Output<eeros::math::Matrix<XBOX_BUTTON_COUNT,1,bool>>& getButtonOut();

  /**
   * @brief Registers a callback invoked on every button state change.
   *
   * The callback receives the button index and its new state (@c true = pressed).
   *
   * @param action  Callable with signature @c void(int buttonIndex, bool pressed)
   */
  inline void on_button(std::function<void(int, bool)> &&action) {
	x.on_button(std::move(action));
  }
			
  double speedScaleFactor{1.0};  ///< Global speed multiplier
  double xScale{0.0001};         ///< Scale factor for x axis
  double yScale{0.0001};         ///< Scale factor for y axis
  double zScale{0.0001};         ///< Scale factor for z axis
  double rScale{0.002};          ///< Scale factor for rotation axis
  double min_x{-0.03},  max_x{0.045};  ///< Clamp range for x axis [m]
  double min_y{-0.03},  max_y{0.03};   ///< Clamp range for y axis [m]
  double min_z{-0.053}, max_z{-0.015}; ///< Clamp range for z axis [m]
  double min_r{-2.8},   max_r{2.8};    ///< Clamp range for rotation [rad]

 protected:
  eeros::hal::XBox x;
  Output<eeros::math::Matrix<XBOX_BUTTON_COUNT,1,bool>> buttonOut;
  eeros::math::Matrix<4,4,double> axisScale;
};

}

#endif /* ORG_EEROS_CONTROL_XBOXINPUT_HPP_ */
