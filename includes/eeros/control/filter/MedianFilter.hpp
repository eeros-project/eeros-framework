#ifndef ORG_EEROS_CONTROL_MEDIANFILTER_HPP_
#define ORG_EEROS_CONTROL_MEDIANFILTER_HPP_

#include <eeros/control/Blockio.hpp>
#include <algorithm>
#include <type_traits>
#include <cmath>


namespace eeros {
namespace control {

/**
 * A median filter (MedianFilter) block is used to filter an input signal. 
 * The output signal value depends on the current and various past
 * input signal values. The median filter algorithm sorts all these values. 
 * The median of these values will be used as output. 
 * 
 * MedianFilter is a class template with one type and one non-type template argument.
 * The type template argument specifies the type which is used for the 
 * values when the class template is instantiated.
 * The non-type template argument specifies the number of stored values.
 * 
 * If the MedianFilter is used with matrices (Matrix, Vector), the filter algorithm
 * will consider all values in the matrice and will not separate them.
 * For example a 3-tuple of a Vector3 instance will be kept together during processing
 * in the MedianFilter.\n
 * If the sort algorithm can not sort the values, they will be left unchanged.
 * 
 * @tparam N - number of considered values
 * @tparam Tval - value type (double - default type)
 * 
 * @since v0.6
 */

template <size_t N, typename Tval = double>
class MedianFilter : public Blockio<1,1,Tval> {
 public:
 
  /**
   * Constructs a MedianFilter instance.
   */
  MedianFilter() {
    zeroInitCurrentValues<Tval>();
  }

  /**
   * Runs the filter algorithm.
   * 
   * Performs the calculation of the filtered output signal value.
   * 
   * Sorts the current and various past input signal values.
   * The median value will be set as output signal value if 
   * the MedianFilter instance is enabled. Otherwise, the output
   * signal value is set to the actual input signal value.
   * 
   * The timestamp value will not be altered.
   *
   * @see enable()
   * @see disable()
   */
  virtual void run() override {
    for(size_t i = 0; i < N-1; i++) {
      currentValues[i] = currentValues[i+1];
    }
    currentValues[N-1] = this->in.getSignal().getValue();
    if(enabled) {
      Tval temp[N]{};
      std::copy(std::begin(currentValues), std::end(currentValues), std::begin(temp));
      std::sort(std::begin(temp), std::end(temp));
      currentMedianValue = temp[medianIndex];
      this->out.getSignal().setValue(currentMedianValue);
    } else {
      this->out.getSignal().setValue(this->in.getSignal().getValue());
    }
    this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
  }

  /**
   * Enables the filter.
   * 
   * If enabled, run() will set the output signal value to the median value
   * which results from sorting the current and the past values. 
   * 
   * @see run()
   */
  virtual void enable() override {
    enabled = true;
  }


  /**
   * Disables the filter.
   * 
   * If disabled, run() will set the output signal to the input signal.
   *
   * @see run()
   */
  virtual void disable() override {
    enabled = false;
  }

  /*
   * Friend operator overload to give the operator overload outside 
   * the class access to the private fields.
   */
  template <size_t No, typename ValT>
  friend std::ostream& operator<<(std::ostream& os, MedianFilter<No,ValT>& filter);

 protected:
  Tval currentValues[N]{};
  Tval currentMedianValue;
  bool enabled{true};
  constexpr static int medianIndex{static_cast<int>(floor(N/2))};

 private:
  template <typename S>
  typename std::enable_if<std::is_arithmetic<S>::value>::type zeroInitCurrentValues() {
    // is zeroed when initialized by default.
  }

  template <typename S>
  typename std::enable_if<!std::is_arithmetic<S>::value>::type zeroInitCurrentValues() {
    for(size_t i = 0; i < N; i++) {
      currentValues[i].zero();
    }
  }
};


/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * MedianFilter instance to an output stream.
 * Does not print a newline control character.
 */
template <size_t N, typename Tval>
std::ostream& operator<<(std::ostream& os, MedianFilter<N,Tval>& filter) {
  os << "Block MedianFilter: '" << filter.getName() << "' is enabled=";
  os << filter.enabled << ", ";
  os << "current median=" << filter.currentMedianValue << ", ";
  os << "medianIndex=" << filter.medianIndex << ", ";
  os << "current values:[" << filter.currentValues[0];
  for(size_t i = 1; i < N; i++){
    os << "," << filter.currentValues[i];
  }
  os << "]";
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_MEDIANFILTER_HPP_ */
