#ifndef ORG_EEROS_CONTROL_CONSTANT_HPP_
#define ORG_EEROS_CONTROL_CONSTANT_HPP_

#include <type_traits>
#include <mutex>
#include <concepts>
#include <limits>
#include <eeros/control/Blockio.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
namespace control {

/**
 * A constant block is used to deliver a constant output signal. Typically its value
 * is set once upon initialization and later altered by the safety system or the sequencer.
 *
 * @tparam T - output signal data type (double - default type)
 * @tparam U - output signal unit type (dimensionless - default type)
 *
 * @since v0.6
 */

template < typename T = double, SIUnit U = SIUnit::create() >
class Constant : public Blockio<0,1,T,T,siunit::generateNSizeArray<0>(),MakeUnitArray<U>::value> {
 public:
  /**
   * Constructs a default constant instance with a value of nan (floating point types) or
   * min (integer types).
   *
   * @see Constant(T v)
   */
  Constant() : value(_clear()) { }

  /**
  * @brief Forwarding constructor.
  *
  * Used for single scalar argument, copy from existing matrix (lvalue)
  * and move vom temporary (rvalue)
  */
  template <typename... Args>
  requires std::constructible_from<T, Args...>
  explicit Constant(Args&&... args) : value(std::forward<Args>(args)...) {}

 /**
  * @brief Initializer list constructor.
  *
  * Needed because initializer_list is a non-deduced context and
  * cannot be captured by the variadic forwarding constructor.
  */
  template <typename V>
  requires std::constructible_from<T, std::initializer_list<V>>
  explicit Constant(std::initializer_list<V> list) : value(list) {}

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Constant(const Constant& other) = delete;

  /**
   * Runs the switch block.
   */
  void run() override {
    std::scoped_lock lock(mtx);
    this->out.getSignal().setValue(value);
    this->out.getSignal().setTimestamp(System::getTimeNs());
  }

  /**
   * Set the value of a constant block to newValue.
   *
   * @param newValue - new value
   */
  virtual void setValue(T newValue) {
    std::scoped_lock lock(mtx);
    value = newValue;
  }

  /**
   * Returns the current value of a constant block.
   *
   * @return - current value
   */
  virtual T getValue () const {
    std::scoped_lock lock(mtx);
    return value;
  }

 protected:
  T value{};
  mutable std::mutex mtx;
  
 private:
  static constexpr T _clear() {
    if constexpr (std::integral<T>) {
      return std::numeric_limits<T>::min();
    }
    else if constexpr (std::floating_point<T>) {
      return std::numeric_limits<T>::quiet_NaN();
    }
    else if constexpr (requires { typename T::value_type; }) {
      T result{};
      if constexpr (std::integral<typename T::value_type>) {
        if constexpr (requires(T t) { t.fill(typename T::value_type{}); }) {
          result.fill(std::numeric_limits<typename T::value_type>::min());
        }
      }
      else if constexpr (std::floating_point<typename T::value_type>) {
        if constexpr (requires(T t) { t.fill(typename T::value_type{}); }) {
          result.fill(std::numeric_limits<typename T::value_type>::quiet_NaN());
        }
      }
      return result;
    }
    else {
      return T{};
    }
  }
};

/********** Print functions **********/
template <typename T, SIUnit U>
std::ostream& operator<<(std::ostream& os, const Constant<T,U>& c) {
  os << "Block constant: '" << c.getName() << "' current val = " << c.getValue(); 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_CONSTANT_HPP_ */
