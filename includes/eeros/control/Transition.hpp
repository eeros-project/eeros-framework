#ifndef ORG_EEROS_CONTROL_TRANSITION_HPP_
#define ORG_EEROS_CONTROL_TRANSITION_HPP_

#include <mutex>
#include <eeros/control/Blockio.hpp>
#include <iostream>

namespace eeros {
namespace control {

template < typename T > class TransitionInBlock;
template < typename T > class TransitionOutBlock;

/**
 * A transition block serves to bring a signal from one timedomain to another. 
 * It consists of two separate blocks, each of them running in one of the 
 * two timedomains which the transition block connects.
 * You have to add the inBlock of this transition block to one timedomain
 * and the outBlock to the second timedomain.
 *
 * @tparam T - signal type (double - default type)
 *
 * @since v1.0
 */

template < typename T = double >
class Transition {
  friend class TransitionInBlock<T>;
  friend class TransitionOutBlock<T>;
 public:
  /**
   * Construct an transition block with one input and one output.
   * Clears the output signal upon creation.
   * The ratio indicates if the transfer happens from slow to fast timedomain (ratio > 1)
   * or from fast to slow timedomain (ratio < 1).
   * For ratio > 1, the transition block usually interpolates. For ratio < 1 the transition
   * block usually filters. If no interpolation or filtering is desired, you can set steady to true.
   *
   * @param ratio - ratio
   * @param steady - true if no interpolation or filtering should happen
   */
  Transition(double ratio, bool steady = false) : inBlock(*this), outBlock(*this), steady(steady), ratio(ratio) {
    if (ratio >= 1.0) {	// slow to fast time domain
      inBlock.up = true;
      outBlock.up = true;
      refresh = false;
      in.clear(); 
      prevIn.clear();
    } else {	// fast to slow time domain
      inBlock.up = false;
      outBlock.up = false;
      bufSize = 1 / ratio;
    }
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Transition(const Transition& s) = delete; 

  /** The block running in the timedomain from which the signal originates */
  TransitionInBlock<T> inBlock;
  /** The block running in the timedomain to which the signal has to be delivered */
  TransitionOutBlock<T> outBlock;
  
 private: 
  std::vector<Signal<T>> buf;
  Signal<T> in, prevIn;
  bool refresh;
  bool steady;
  double ratio;
  uint32_t bufSize;
  std::mutex mtx;
};

template < typename T = double >
class TransitionInBlock : public Blockio<1,0,T> {
  friend class Transition<T>;
 public:
  TransitionInBlock(Transition<T>& c) : container(c) { }
  
  virtual void run() { 
    if (container.steady) {
      container.mtx.lock();
      container.in = this->getIn().getSignal(); 
      container.mtx.unlock();
    } else {
      if (up) {	// up
        container.mtx.lock();
        container.prevIn = container.in;
        container.in = this->getIn().getSignal(); 
        container.refresh = true;
        container.mtx.unlock();
      } else {	//down
        container.mtx.lock();
        auto val = this->getIn().getSignal(); 
        container.buf.push_back(val);
        container.mtx.unlock();
      }
    }
  }
  
 protected:
  bool up;
  Transition<T>& container;
};

template < typename T = double >
class TransitionOutBlock : public Blockio<1,1,T> {
  friend class Transition<T>;
 public:
  TransitionOutBlock(Transition<T>& c) : container(c), count(0) {in.clear(); prevIn.clear();}
  
  virtual void run() { 
    if (container.steady) {
      container.mtx.lock();
      this->getOut().getSignal().setValue(container.in.getValue()); 
      this->getOut().getSignal().setTimestamp(container.in.getTimestamp());
      container.mtx.unlock();
    } else {
      if (up) {	// up
        if (container.refresh) {
          container.mtx.lock();
          prevIn = container.prevIn;
          in = container.in;
          container.refresh = false;
          container.mtx.unlock();
          count = 0;
          dVal = (in.getValue() - prevIn.getValue()) / container.ratio;
          dTime= (in.getTimestamp() - prevIn.getTimestamp()) / container.ratio;
        }
        T val = prevIn.getValue() + dVal * count;
        this->getOut().getSignal().setValue(val);
        timestamp_t time = prevIn.getTimestamp() + count * dTime;
        this->getOut().getSignal().setTimestamp(time);
        count++;
      } else {	//down
        auto time = this->getIn().getSignal().getTimestamp();
        container.mtx.lock();
        std::size_t i = 0;
        while (i < container.buf.size() && time > container.buf[i].getTimestamp()) i++;
        if (i > 0) i--;
        Signal<T> sig = container.buf[i];
        container.buf.clear();
        container.mtx.unlock();
        this->getOut().getSignal().setValue(sig.getValue());
        this->getOut().getSignal().setTimestamp(sig.getTimestamp());
      }
    }
  }
  
 protected:
  bool up;
  Transition<T>& container;
  Signal<T> prevIn, in;
  T dVal;
  double dTime;
  uint32_t count;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * transition block instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, Transition<T>& t) {
  os << "Block transition: '" << t.getName() << "'"; 
        return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_TRANSITION_HPP_ */
