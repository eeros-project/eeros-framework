#ifndef ORG_EEROS_CONTROL_SIGNAL_HPP_
#define ORG_EEROS_CONTROL_SIGNAL_HPP_

#include <string>
#include <list>
#include <type_traits>
#include <limits>
#include <eeros/types.hpp>
#include <eeros/control/SignalInterface.hpp>

namespace eeros {
namespace control {
    
extern uint16_t signalCounter;
      
/**
 * A signal comprises several properties such as a value and a timestamp.
 * It is used to transport information between blocks of a control system.
 *
 * @tparam T - signal type (double - default type)
 * @since v0.4
 */

template < typename T = double >
class Signal : public SignalInterface {
 public:
  /**
   * Constructs a signal instance.
   */
  Signal() {
   id = signalCounter++;
  }
      
  /**
   * Gets the unique id of this signal.
   * 
   * @return id
   */
  virtual sigid_t getId() const {
    return static_cast<sigid_t>(id) << 16;
  }
      
  /**
   * Gets the name of this signal.
   * 
   * @return name
   */
  virtual std::string getName() const {
    return name;
  }
      
  /**
   * Sets the name of this signal.
   * 
   * @param name - name of the signal
   */
  virtual void setName(std::string name) {
    this->name = name;
  }
      
      virtual std::string getLabel() const {
// 				std::stringstream label;
// 				label << '#' << id << '.' << index << ": " << getName(index);
// 				if(getAffiliation(index) != "") {
// 					label << " (" << getAffiliation(index) << ')';
// 				}
// 				if(getUnit(index) != "") {
// 					label << " [" << getUnit(index) << ']';
// 				}
// 				return label.str();
        return name; // TODO
      }
      
  /**
   * Gets the value of this signal.
   * 
   * @return value
   */
  virtual T getValue() const {
    return value;
  }
      
  /**
   * Sets the value of this signal.
   * 
   * @param newValue - value of the signal
   */
  virtual void setValue(T newValue) {
    value = newValue;
  }
      
  /**
   * Gets the timestamp of this signal.
   * 
   * @return timestamp
   */
  virtual timestamp_t getTimestamp() const {
    return timestamp;
  }
      
  /**
   * Sets the timestamp of this signal.
   * 
   * @param newTimestamp - timestamp of the signal
   */
  virtual void setTimestamp(timestamp_t newTimestamp) {
    timestamp = newTimestamp;
  }
      
  /**
   * Clears the signal to NaN.
   */
  virtual void clear() {
    if constexpr (std::integral<T>) {
      value = std::numeric_limits<T>::min();
    } else if constexpr (std::floating_point<T>) {
      value = std::numeric_limits<T>::quiet_NaN();
    } else if constexpr (requires { typename T::value_type; }) {
      using VT = typename T::value_type;
      if constexpr (std::integral<VT>) {
        value.fill(std::numeric_limits<VT>::min());
      } else if constexpr (std::floating_point<VT>) {
        value.fill(std::numeric_limits<VT>::quiet_NaN());
      } else {
        // nested compound type (e.g. Matrix<2,1,Vector2>):
        // default-initialize each element
        value.fill(VT{});
      }
    } else {
      value = T{};
    }
    timestamp = 0;
  }
      
  Signal<T>& operator= (Signal<T> right) {
    value = right.value;
    timestamp = right.timestamp;
    return *this;
  }
      
  Signal<T>& operator= (T right) {
    value = right;
    return *this;
  }
      
  static Signal<T>& getIllegalSignal() {
    return illegalSignal;
  }
      
  static std::list<SignalInterface*> getSignalList() {
    return signalList;
  }
      
  static SignalInterface* getSignalById(sigid_t id) {
    std::list<SignalInterface*>::iterator i = signalList.begin();
    while(i != signalList.end()) {
      if((*i)->getId() == id) {
        return (*i);
      }
      i++;
    }
    return NULL;
  }
      
 protected:
  T value; /** The value carries the signal value, it can be of any physical type */
  timestamp_t timestamp; /** The timestamp marks the time when this signal was captured */
  sigid_t id; /** Each signal has an unique id which is assigned automatically upon creation */
  std::string name; /** Each signal can be named */
    
 private:
  static std::list<SignalInterface*> signalList;
  static Signal<T> illegalSignal;
};
    
template < typename T>
std::list<SignalInterface*> Signal<T>::signalList;
    
template < typename T>
Signal<T> Signal<T>::illegalSignal;
  
/********** Print functions **********/
template <typename T>
std::ostream& operator<<(std::ostream& os, Signal<T>& signal) {
  os << "Signal: '" << signal.getName() << "' timestamp = " << signal.getTimestamp() << " value = " << signal.getValue(); 
  return os;
}
template <typename T>
std::ostream& operator<<(std::ostream& os, Signal<T>* signal) {
  os << "Signal: '" << signal->getName() << "' timestamp = " << signal->getTimestamp() << " value = " << signal->getValue(); 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_SIGNAL_HPP_ */
