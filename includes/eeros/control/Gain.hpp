#ifndef ORG_EEROS_CONTROL_GAIN_HPP_
#define ORG_EEROS_CONTROL_GAIN_HPP_

#include <eeros/control/Blockio.hpp>
#include <type_traits>
#include <concepts>
#include <memory>
#include <mutex>
#include <cmath>
#include <limits>
#include <ostream>


namespace eeros::control {

/**
 * A gain block is used to amplify an input signal. This is basically done by
 * multiplying the gain with the input signal value.
 * The following term represents the operation performed in this block.
 *
 * output = gain * input
 *
 * Gain is a class template with two type and one non-type template arguments.
 * The two type template arguments specify the types which are used for the
 * output type and the gain type when the class template is instanciated.
 * The non-type template argument specifies if the multiplication will be done
 * element wise in case the gain is used with matrices.
 *
 * A gain block is suitable for use with multiple threads. However,
 * enabling/disabling of the gain and the smooth change feature
 * is not synchronized.
 *
 * @tparam Tout - input and output signal data type (double - default type)
 * @tparam Tgain - gain data type (double - default type)
 * @tparam elementWise - amplify element wise (false - default value)
 *
 * @since v0.6
 */

template< typename Tout = double, typename Tgain = double, bool elementWise = false >
class Gain : public Blockio<1,1,Tout,Tout> {
 public:
  /**
   * Constructs a default gain instance with a gain of 1.\n
   * Calls Gain(Tgain c).
   *
   * @see Gain(Tgain c)
   */
  Gain() : Gain(Tgain(1)) {}

  /**
   * Constructs a gain instance with a gain of the value of the parameter c.\n
   * Calls Gain(Tgain c, Tgain maxGain, Tgain minGain).\n
   *
   * @see Gain(Tgain c, Tgain maxGain, Tgain minGain)
   *
   * @param c - initial gain value
   */
  Gain(Tgain c) {
    gain = c;
    targetGain = c;
    gainDiff = Tgain{};
    resetMinMaxGain();
  }

  /**
   * Constructs a gain instance with a gain of the value of the parameter c,
   * a maximum gain of maxGain and a minimum gain of minGain.\n
   * Sets the target gain to the value of the parameter c.\n
   * Sets the gain diff to 0.
   *
   * @param c - initial gain value
   * @param maxGain - initial maximum gain value
   * @param minGain - initial minimum gain value
   */
  Gain(Tgain c, Tgain maxGain, Tgain minGain) {
    gain = c;
    this->maxGain = maxGain;
    this->minGain = minGain;
    targetGain = gain;
    gainDiff = Tgain{};
  }


  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Gain(const Gain& s) = delete;

  /**
   * Runs the amplification algorithm.
   *
   * Performs the smooth change if smooth change is enabled with enableSmoothChange(bool).
   * A smooth change of the gain is performed by adding or subtracting a gain differential
   * specifiable by setGainDiff(Tgain).
   *
   * Checks if gain is in the band in between minGain and maxGain or correct it otherwise.
   * The correction is done by setting the maxGain or minGain to gain.
   *
   * Sets the output signal value to the amplified value calculated by multiplying gain * input if
   * the gain instance is enabled by enable().
   *
   * @see enableSmoothChange(bool)
   * @see enableRootCurveChange(bool)
   * @see setGainDiff(Tgain)
   * @see enable()
   * @see disable()
   */
  void run() override {
    std::lock_guard<std::mutex> lock(mtx);

    if (smoothChange) {
      if (gain < targetGain) {
        gain += gainDiff;
        if (gain > targetGain) { // overshoot case.
          gain = targetGain;
        }
      }

      if (gain > targetGain) {
        gain -= gainDiff;
        if (gain < targetGain) {
          gain = targetGain;
        }
      }
    }

    if (gain > maxGain) { // if diff will cause gain to be too large.
      gain = maxGain;
    }

    if (gain < minGain) {
      gain = minGain;
    }

    if (enabled) {
      if (parabolic) this->out.getSignal().setValue(calculateParabolic<Tout,Tgain>(this->in.getSignal().getValue()));
      else this->out.getSignal().setValue(calculate<Tout>(this->in.getSignal().getValue()));
    } else {
      this->out.getSignal().setValue(this->in.getSignal().getValue());
    }

    this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
  }

  /**
   * Enables the gain.
   *
   * If enabled, run() will set the output signal to the amplified value calculated by multiplying gain * input.
   *
   * Does not enable smooth change. This is done by calling enableSmoothChange(bool).
   *
   * @see run()
   * @see enableSmoothChange(bool)
   */
  void enable() override {
    enabled = true;
  }

  /**
   * Disables the gain.
   *
   * If disabled, run() will set the output signal to the input signal.
   *
   * Does not disable smooth change. This is done by calling enableSmoothChange(bool).
   *
   * @see run()
   * @see enableSmoothChange(bool)
   */
  void disable() override {
    enabled = false;
  }

  /**
   * Enables or disables a smooth change of the gain.
   *
   * If enabled, run() will perform a smooth change of the gain value.
   *
   * Does not enable or disable the gain. This is done by calling enable() and disable() respectively.
   *
   * @param enable - enables or disables a smooth change of the gain
   *
   * @see run()
   * @see enable()
   * @see disable()
   */
  virtual void enableSmoothChange(bool enable) {
    smoothChange = enable;
  }

  /**
   * Enables or disables a parabolic profile for the gain
   *
   * If enabled, run() will calculate and apply a parabolic profile to the gain above a
   * given input value.
   *
   * Does not enable or disable the gain. This is done by calling enable() and disable() respectively.
   *
   * @param enable - enables or disables the root curve option
   *
   * @see setParabolicGainParams()
   */
  virtual void enableParabolicGain(bool enable) {
    parabolic = enable;
  }

  /**
   * Sets the gain value if smooth change is disabled and c is in the band in between minGain and maxGain.
   *
   * Sets the target gain value if smooth change is enabled and c is in the band in between minGain and maxGain.
   *
   * Does not change gain or target gain value otherwise.
   *
   * @param c - gain value
   */
  virtual void setGain(Tgain c) {
    std::lock_guard<std::mutex> lock(mtx);

    if (c <= maxGain && c >= minGain) {
      if (smoothChange) {
        targetGain = c;
      } else {
        gain = c;
      }
    }
  }

  /**
   * Sets the maximum allowed gain.
   *
   * @param maxGain - maximum allowed gain value
   */
  virtual void setMaxGain(Tgain maxGain) {
    std::lock_guard<std::mutex> lock(mtx);
    this->maxGain = maxGain;
  }

  /**
   * Sets the minimum allowed gain.
   *
   * @param minGain - minimum allowed gain value
   */
  virtual void setMinGain(Tgain minGain) {
    std::lock_guard<std::mutex> lock(mtx);
    this->minGain = minGain;
  }

  /**
   * Sets the gain differential needed by run() to perform a smooth gain change.
   *
   * @param gainDiff - gain differential
   */
  virtual void setGainDiff(Tgain gainDiff) {
    std::lock_guard<std::mutex> lock(mtx);
    this->gainDiff = gainDiff;
  }

  /**
   * Sets the limit for the input value above which the gain follows a parabolic behavior.
   *
   * @param parabolicSwitchPoint - input limit
   */
  virtual void setParabolicGainParams(Tout parabolicSwitchPoint) {
    std::lock_guard<std::mutex> lock(mtx);
    this->parabolicSwitchPoint = parabolicSwitchPoint;
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename Xout, typename Xgain, bool XelementWise>
  friend std::ostream &operator<<(std::ostream &os, const Gain<Xout, Xgain, XelementWise> &gain);

 protected:
  Tgain gain{};
  Tgain maxGain{};
  Tgain minGain{};
  Tgain targetGain{};
  Tgain gainDiff{};
  bool enabled{true};
  bool smoothChange{false};
  bool parabolic{false};
  Tout parabolicSwitchPoint;
  std::mutex mtx;

 private:
  template<typename R>
  R calculate(R value) {
    if constexpr (!elementWise) {
      return gain * value;
    } else {
      static_assert(std::is_compound_v<R>,
                    "A gain block with element wise amplification must use matrices!");
      return value.multiplyElementWise(gain);
    }
  }

  template<typename R, typename S>
  R calculateParabolic(R value) {
    Tout outVal{};
    if constexpr (std::is_arithmetic_v<R>) { // Scalar value (arithmetic types)
      if (std::fabs(value) > parabolicSwitchPoint) {
        if (value >= 0) {
          outVal = gain * std::sqrt(parabolicSwitchPoint * (2 * value - parabolicSwitchPoint));
        } else {
          outVal = gain * -std::sqrt(parabolicSwitchPoint * (-2 * value - parabolicSwitchPoint));
        }
      } else {
        outVal = gain * value;
      }
    } else if constexpr (std::is_compound_v<R>) { // Compound value (vectors/matrices)
      if constexpr (std::is_arithmetic_v<S>) { // Scalar gain with compound value
        if constexpr (elementWise) {
          static_assert(!elementWise,
                        "A gain block with scalar gain factor must not use elementwise multiplication");
        } else {
          for (unsigned int i = 0; i < value.size(); i++) {
            if (std::fabs(value[i]) > parabolicSwitchPoint[i]) {
              if (value[i] >= 0) {
                outVal[i] = gain * std::sqrt(parabolicSwitchPoint[i] * (2 * value[i] - parabolicSwitchPoint[i]));
              } else {
                outVal[i] = gain * -std::sqrt(parabolicSwitchPoint[i] * (-2 * value[i] - parabolicSwitchPoint[i]));
              }
            } else {
              outVal[i] = gain * value[i];
            }
          }
        }
      } else if constexpr (std::is_compound_v<S>) { // Compound gain with compound value
        if constexpr (elementWise) {
          // Element-wise with matrix gain
          for (unsigned int i = 0; i < value.size(); i++) {
            if (std::fabs(value[i]) > parabolicSwitchPoint[i]) {
              if (value[i] >= 0) {
                outVal[i] = gain[i] * std::sqrt(parabolicSwitchPoint[i] * (2 * value[i] - parabolicSwitchPoint[i]));
              } else {
                outVal[i] = gain[i] * -std::sqrt(parabolicSwitchPoint[i] * (-2 * value[i] - parabolicSwitchPoint[i]));
              }
            } else {
              outVal[i] = gain[i] * value[i];
            }
          }
        }
      }
    }
    return outVal;
  }

  void resetMinMaxGain() {
    if constexpr (std::integral<Tgain>) {
      minGain = std::numeric_limits<Tgain>::min();
      maxGain = std::numeric_limits<Tgain>::max();
    } else if constexpr (std::floating_point<Tgain>) {
      minGain = std::numeric_limits<Tgain>::lowest();
      maxGain = std::numeric_limits<Tgain>::max();
    } else if constexpr (requires { typename Tgain::value_type; }) {
      using ValueType = typename Tgain::value_type;
      if constexpr (std::integral<ValueType>) {
        for (unsigned int i = 0; i < minGain.size(); i++) {
          minGain[i] = std::numeric_limits<ValueType>::min();
          maxGain[i] = std::numeric_limits<ValueType>::max();
        }
      } else if constexpr (std::floating_point<ValueType>) {
        for (unsigned int i = 0; i < minGain.size(); i++) {
          minGain[i] = std::numeric_limits<ValueType>::lowest();
          maxGain[i] = std::numeric_limits<ValueType>::max();
        }
      }
    }
  }
};


/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Gain instance to an output stream.\n
 * Does not print a newline control character.
 */
template<typename Tout, typename Tgain, bool elementWise>
std::ostream &operator<<(std::ostream &os, const Gain<Tout, Tgain, elementWise> &gain) {
  os << "Block Gain: '" << gain.getName() << "' is enabled=" << gain.enabled << ", gain=" << gain.gain << ", ";
  os << "smoothChange=" << gain.smoothChange << ", minGain=" << gain.minGain << ", maxGain=" << gain.maxGain;
  os << ", targetGain=" << gain.targetGain << ", gainDiff=" << gain.gainDiff;
  os << ", parabolic=" << gain.parabolic << ", parabolicSwitchPoint=" << gain.parabolicSwitchPoint;
  return os;
}

}

#endif /* ORG_EEROS_CONTROL_GAIN_HPP_ */
