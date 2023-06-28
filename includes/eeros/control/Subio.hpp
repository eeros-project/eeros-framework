#ifndef ORG_EEROS_CONTROL_SUBIO_HPP_
#define ORG_EEROS_CONTROL_SUBIO_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>
#include <functional>

namespace eeros {
namespace control {

/**
 * Base class for all subsystem blocks with inputs and outputs.
 *
 * Extend this class and override the run method to implement any
 * given subsystem. 
 * 
 * @tparam N - number of inputs
 * @tparam M - number of outputs
 * @tparam Tin - input type (double - default type)
 * @tparam Tout - output type (double - default type)
 * @since v1.4.1
 */

template < uint8_t N, uint8_t M, typename Tin = double, typename Tout = Tin >
class Subio : public Block {
 public:
  /**
   * Construct a subsystem block with inputs and outputs. 
   * Clears the output signal.
   */
  Subio() { 
    for (uint8_t i = 0; i < N; i++) in[i].Input<Tin>::setOwner(this);
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 

  /**
   * Get an input of the subsystem block.
   * 
   * @param index - index of the input
   * @return input
   */
  virtual Input<Tin>& getIn(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return in[index];
  }

  /**
   * Get an output of the block.
   * 
   * @param index - index of the input
   * @return output
   */
  virtual Output<Tout>& getOut(uint8_t index) {
    if (index >= M) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return *out[index];
  }

  /**
   * Set the output of the subsystem block.
   *
   * @param out - output
   * @param index - index of the output
   */
  virtual void setOut(Output<Tout>& out, uint8_t index) {
    this->out[index] = &out;
    this->out[index]->setOwner(this);
    this->out[index]->getSignal().clear();
  }

 protected:
  InputSub<Tin> in[N];
  Output<Tout>* out[M];
};

/**
 * Spezialization for several inputs and 1 output
 */
template < uint8_t N, typename Tin, typename Tout >
class Subio<N,1,Tin,Tout> : public Block {
 public:
  /**
   * Construct a subsystem block with several inputs and one output. 
   * Clears the output signal.
   */
  Subio() {
    for (uint8_t i = 0; i < N; i++) in[i].Input<Tin>::setOwner(this);
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 

  /**
   * Get an input of the subsystem block.
   * 
   * @param index - index of the input
   * @return input
   */
  virtual Input<Tin>& getIn(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return in[index];
  }

  /**
   * Get the output of the subsystem block.
   * 
   * @return output
   */
  virtual Output<Tout>& getOut() {
    return *out;
  }

  /**
   * Set the output of the subsystem block.
   *
   * @param out - output
   */
  virtual void setOut(Output<Tout>& out) {
    this->out = &out;
    this->out->setOwner(this);
    this->out->getSignal().clear();
  }

 protected:
  InputSub<Tin> in[N];
  Output<Tout>* out;
};

/**
 * Spezialization for several inputs and no output
 */
template < uint8_t N, typename Tin >
class Subio<N,0,Tin> : public Block {
 public:
  /**
   * Construct a subsystem block with several inputs and no output. 
   * Clears the output signal.
   */
  Subio() { 
    for (uint8_t i = 0; i < N; i++) in[i].Input<Tin>::setOwner(this);
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 

  /**
   * Get an input of the subsystem block.
   * 
   * @param index - index of the input
   * @return input
   */
  virtual Input<Tin>& getIn(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return in[index];
  }

 protected:
  InputSub<Tin> in[N];
};

/**
 * Spezialization for 1 input and several outputs
 */
template < uint8_t M, typename Tin, typename Tout >
class Subio<1,M,Tin,Tout> : public Block {
 public:
  /**
   * Construct a subsystem block with one input and several outputs. 
   * Clears the output signal.
   */
  Subio() : in(this) { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 

  /**
   * Get the input of the subsystem block.
   * 
   * @return input
   */
  virtual Input<Tin>& getIn() {
    return in;
  }

  /**
   * Get an output of the subsystem block.
   * 
   * @param index - index of the input
   * @return output
   */
  virtual Output<Tout>& getOut(uint8_t index) {
    if (index >= M) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return *out[index];
  }

  /**
   * Set the output of the subsystem block.
   *
   * @param out - output
   * @param index - index of the output
   */
  virtual void setOut(Output<Tout>& out, uint8_t index) {
    this->out[index] = &out;
    this->out[index]->setOwner(this);
    this->out[index]->getSignal().clear();
  }

 protected:
  InputSub<Tin> in;
  Output<Tout>* out[M];
};

/**
 * Spezialization for 1 input and 1 output
 */
template < typename Tin, typename Tout >
class Subio<1,1,Tin,Tout> : public Block {
 public:
  /**
   * Construct a subsystem block with one input and one output. 
   * Clears the output signal.
   */
  Subio() : in(this) { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 

  /**
   * Get the input of the subsystem block.
   * 
   * @return input
   */
  virtual Input<Tin>& getIn() {
    return in;
  }

  /**
   * Get the output of the subsystem block.
   * 
   * @return output
   */
  virtual Output<Tout>& getOut() {
    return *out;
  }

  /**
   * Set the output of the subsystem block.
   *
   * @param out - output
   */
  virtual void setOut(Output<Tout>& out) {
    this->out = &out;
    this->out->setOwner(this);
    this->out->getSignal().clear();
  }

 protected:
  InputSub<Tin> in;
  Output<Tout>* out;
};

/**
 * Spezialization for 1 input and no output
 */
template < typename Tin >
class Subio<1,0,Tin> : public Block {
 public:
  /**
   * Construct a subsystem block with one input and no output. 
   */
  Subio() : in(this) { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 

  /**
   * Get the input of the subsystem block.
   * 
   * @return input
   */
  virtual Input<Tin>& getIn() {
    return in;
  }

 protected:
  InputSub<Tin> in;
};

/**
 * Spezialization for no input and several outputs
 */
template < uint8_t M, typename Tout >
class Subio<0,M,Tout> : public Block {
 public:
  /**
   * Construct a subsystem block with no input and several outputs. 
   * Clears the output signals.
   */
  Subio() { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 

  /**
   * Get an output of the subsystem block.
   * 
   * @param index - index of the input
   * @return output
   */
  virtual Output<Tout>& getOut(uint8_t index) {
    if (index >= M) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return *out[index];
  }

  /**
   * Set the output of the subsystem block.
   *
   * @param out - output
   * @param index - index of the output
   */
  virtual void setOut(Output<Tout>& out, uint8_t index) {
    this->out[index] = &out;
    this->out[index]->setOwner(this);
    this->out[index]->getSignal().clear();
  }

 protected:
  Output<Tout>* out[M];
};

/**
 * Spezialization for no input and one output
 */
template < typename Tout >
class Subio<0,1,Tout> : public Block {
 public:
  /**
   * Construct a subsystem block with no input and one output. 
   * Clears the output signals.
   */
  Subio() : out(this) { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 

  /**
   * Get the output of the subsystem block.
   * 
   * @return output
   */
  virtual Output<Tout>& getOut() {
    return *out;
  }

  /**
   * Set the output of the subsystem block.
   *
   * @param out - output
   */
  virtual void setOut(Output<Tout>& out) {
    this->out = &out;
    this->out->setOwner(this);
    this->out->getSignal().clear();
  }
 protected:
  Output<Tout>* out;
};

/**
 * Spezialization for no input and no output
 */
template < >
class Subio<0,0> : public Block {
 public:
  /**
   * Construct a subsystem block with no input and no output. 
   */
  Subio() { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Subio(const Subio& s) = delete; 
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * subsystem block instance to an output stream.\n
 * Does not print a newline control character.
 */
template < uint8_t N, uint8_t M, typename Tin = double, typename Tout = Tin >
std::ostream& operator<<(std::ostream& os, Subio<N,M,Tin,Tout>& b) {
  os << "Subsystem block: '" << b.getName() << "'"; 
  return os;
}

}
}
#endif /* ORG_EEROS_CONTROL_SUBIO_HPP_ */
