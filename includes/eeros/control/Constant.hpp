#ifndef ORG_EEROS_CONTROL_CONSTANT_HPP_
#define ORG_EEROS_CONTROL_CONSTANT_HPP_

#include <type_traits>
#include <mutex>
#include <eeros/control/Blockio.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
namespace control {

/**
 * A constant block is used to deliver a constant output signal. Typically its value
 * is set once upon initialization and later altered by the safety system or the sequencer.
 *
 * @tparam T - value type (double - default type)
 *
 * @since v0.6
 */

template < typename T = double >
class Constant : public Blockio<0,1,T> {
 public:
  /**
   * Constructs a default constant instance with a value of nan (floating point types) or
   * min (integer types).
   *
   * @see Constant(T v)
   */
  Constant() {
    _clear<T>();
  }
  
  /**
   * Constructs a constant instance with a initial value of v.
   *
   * @param v - initial value
   */
  Constant(T v) : value(v) { }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Constant(const Constant& other) = delete;

  /**
   * Runs the switch block.
   */
  virtual void run() {
    std::lock_guard<std::mutex> lock(mtx);
    this->out.getSignal().setValue(value);
    this->out.getSignal().setTimestamp(System::getTimeNs());
  }
  
  /**
   * Set the value of a constant block to newValue.
   *
   * @param newValue - new value
   */
  virtual void setValue(T newValue) {
    std::lock_guard<std::mutex> lock(mtx);
    value = newValue;
  }

  /**
   * Returns the current value of a constant block.
   *
   * @return - current value
   */
  virtual T getValue () const {
    return value;
  }

protected:
  T value;
  std::mutex mtx;
  
private:
  template <typename S> typename std::enable_if<std::is_integral<S>::value>::type _clear() {
    value = std::numeric_limits<int32_t>::min();
  }
  template <typename S> typename std::enable_if<std::is_floating_point<S>::value>::type _clear() {
    value = std::numeric_limits<double>::quiet_NaN();
  }
  template <typename S> typename std::enable_if<std::is_compound<S>::value && std::is_integral<typename S::value_type>::value>::type _clear() {
    value.fill(std::numeric_limits<int32_t>::min());
  }
  template <typename S> typename std::enable_if<std::is_compound<S>::value && std::is_floating_point<typename S::value_type>::value>::type _clear() {
    value.fill(std::numeric_limits<double>::quiet_NaN());
  }
  template<typename S> typename std::enable_if<std::is_enum<S>::value>::type _clear() {
    value = static_cast<S>(0);
  }
};

/********** Print functions **********/
template <typename T>
std::ostream& operator<<(std::ostream& os, Constant<T>& c) {
  os << "Block constant: '" << c.getName() << "' current val = " << c.getValue(); 
        return os;
}
};
};

#endif /* ORG_EEROS_CONTROL_CONSTANT_HPP_ */
