#ifndef ORG_EEROS_CONTROL_BLOCKIO_HPP_
#define ORG_EEROS_CONTROL_BLOCKIO_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>
#include <functional>

namespace eeros {
namespace control {

/**
 * Base class for all blocks with inputs and outputs.
 *
 * Extend this class and override the run method to implement any
 * given algorithm. 
 * 
 * Alternatively, an algorithm can be set directly 
 * when creating such a block. Choose this method when the algorithm 
 * is simple and one wants to avoid using several other blocks doing 
 * a simple algorithm, e.g. adding a offset and scale to a signal.
 * 
 * Define such a block with an example algorithm as follows:
 * Blockio<2,1,Vector2,Vector2> block([&]() {
 *   auto val = (block.getIn(0).getSignal().getValue() + 0.5) * 2;
 *   val[0] *= -1.0;
 *   val += block.getIn(1).getSignal().getValue() + 1.0;
 *   block.getOut().getSignal().setValue(val);
 *   block.getOut().getSignal().setTimestamp(gen.getIn(0).getSignal().getTimestamp());
 * });
* 
 * @tparam N - number of inputs
 * @tparam M - number of outputs
 * @tparam Tin - input type (double - default type)
 * @tparam Tout - output type (double - default type)
 * @since v1.2.1
 */

template < uint8_t N, uint8_t M, typename Tin = double, typename Tout = Tin >
class Blockio : public Block {
 public:
  /**
   * Construct a block with inputs and outputs. 
   * Clears the output signal.
   */
  Blockio() : Blockio([](){}) { }

  /**
   * Construct a block with inputs and outputs.
   * Clears the output signal.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : func(f) {
    for (uint8_t i = 0; i < N; i++) in[i].setOwner(this);
    for (uint8_t i = 0; i < M; i++) {
      out[i].setOwner(this);
      out[i].getSignal().clear();
    }
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

  /**
   * Get an input of the block.
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
    return out[index];
  }

 protected:
  Input<Tin> in[N];
  Output<Tout> out[M];

 private:
  std::function<void()> func;
};

/**
 * Spezialization for several inputs and 1 output
 */
template < uint8_t N, typename Tin, typename Tout >
class Blockio<N,1,Tin,Tout> : public Block {
 public:
  /**
   * Construct an block with several inputs and one output. 
   * Clears the output signal.
   */
  Blockio() : Blockio([](){}) { }

  /**
   * Construct an block with several inputs and one output.
   * Clears the output signal.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : out(this), func(f) {
    for (uint8_t i = 0; i < N; i++) in[i].setOwner(this);
    out.getSignal().clear();
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

  /**
   * Get an input of the block.
   * 
   * @param index - index of the input
   * @return input
   */
  virtual Input<Tin>& getIn(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return in[index];
  }

  /**
   * Get the output of the block.
   * 
   * @return output
   */
  virtual Output<Tout>& getOut() {
    return out;
  }

 protected:
  Input<Tin> in[N];
  Output<Tout> out;

 private:
  std::function<void()> func;
};

/**
 * Spezialization for several inputs and no output
 */
template < uint8_t N, typename Tin >
class Blockio<N,0,Tin> : public Block {
 public:
  /**
   * Construct an block with several inputs and one output. 
   * Clears the output signal.
   */
  Blockio() : Blockio([](){}) { }

  /**
   * Construct an block with several inputs and one output.
   * Clears the output signal.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : func(f) {
    for (uint8_t i = 0; i < N; i++) in[i].setOwner(this);
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

  /**
   * Get an input of the block.
   * 
   * @param index - index of the input
   * @return input
   */
  virtual Input<Tin>& getIn(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return in[index];
  }

 protected:
  Input<Tin> in[N];

 private:
  std::function<void()> func;
};

/**
 * Spezialization for 1 input and several outputs
 */
template < uint8_t M, typename Tin, typename Tout >
class Blockio<1,M,Tin,Tout> : public Block {
 public:
  /**
   * Construct an block with one input and several outputs. 
   * Clears the output signal.
   */
  Blockio() : Blockio([](){}) { }

  /**
   * Construct an block with one input and several outputs.
   * Clears the output signal.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : in(this), func(f) {
    for (uint8_t i = 0; i < M; i++) {
      out[i].setOwner(this);
      out[i].getSignal().clear();
    }
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

  /**
   * Get the input of the block.
   * 
   * @return input
   */
  virtual Input<Tin>& getIn() {
    return in;
  }

  /**
   * Get an output of the block.
   * 
   * @param index - index of the input
   * @return output
   */
  virtual Output<Tout>& getOut(uint8_t index) {
    if (index >= M) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return out[index];
  }

 protected:
  Input<Tin> in;
  Output<Tout> out[M];

 private:
  std::function<void()> func;
};

/**
 * Spezialization for 1 input and 1 output
 */
template < typename Tin, typename Tout >
class Blockio<1,1,Tin,Tout> : public Block {
 public:
  /**
   * Construct an block with one input and one output. 
   * Clears the output signal.
   */
  Blockio() : Blockio([](){}) { }

  /**
   * Construct an block with one input and one output.
   * Clears the output signal.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : in(this), out(this), func(f) { 
    out.getSignal().clear();
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

  /**
   * Get the input of the block.
   * 
   * @return input
   */
  virtual Input<Tin>& getIn() {
    return in;
  }

  /**
   * Get the output of the block.
   * 
   * @return output
   */
  virtual Output<Tout>& getOut() {
    return out;
  }

 protected:
  Input<Tin> in;
  Output<Tout> out;

 private:
  std::function<void()> func;
};

/**
 * Spezialization for 1 input and no output
 */
template < typename Tin >
class Blockio<1,0,Tin> : public Block {
 public:
  /**
   * Construct an block with one input and no output. 
   */
  Blockio() : Blockio([](){}) { }

  /**
   * Construct an block with one input and no output.
   * Clears the output signal.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : in(this), func(f) { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

  /**
   * Get the input of the block.
   * 
   * @return input
   */
  virtual Input<Tin>& getIn() {
    return in;
  }

 protected:
  Input<Tin> in;

 private:
  std::function<void()> func;
};

/**
 * Spezialization for no input and several outputs
 */
template < uint8_t M, typename Tout >
class Blockio<0,M,Tout> : public Block {
 public:
  /**
   * Construct an block with no input and several outputs. 
   * Clears the output signals.
   */
  Blockio() : Blockio([](){}) { }

  /**
   * Construct an block with no input and several outputs.
   * Clears the output signals.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : func(f) {
    for (uint8_t i = 0; i < M; i++) {
      out[i].setOwner(this);
      out[i].getSignal().clear();
    }
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

  /**
   * Get an output of the block.
   * 
   * @param index - index of the input
   * @return output
   */
  virtual Output<Tout>& getOut(uint8_t index) {
    if (index >= M) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return out[index];
  }

 protected:
  Output<Tout> out[M];

 private:
  std::function<void()> func;
};

/**
 * Spezialization for no input and one output
 */
template < typename Tout >
class Blockio<0,1,Tout> : public Block {
 public:
  /**
   * Construct an block with no input and one output. 
   * Clears the output signals.
   */
  Blockio() : Blockio([](){}) { }

  /**

   * Construct an block with no input and one output.
   * Clears the output signal.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : func(f) {
    out.setOwner(this);
    out.getSignal().clear();
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

  /**
   * Get the output of the block.
   * 
   * @return output
   */
  virtual Output<Tout>& getOut() {
    return out;
  }

 protected:
  Output<Tout> out;

 private:
  std::function<void()> func;
};

/**
 * Spezialization for no input and no output
 */
template < >
class Blockio<0,0> : public Block {
 public:
  /**
   * Construct an block with no input and no output. 
   */
  Blockio() : Blockio([](){}) { }

  /**

   * Construct an block with no input and no output.
   * The block will run a given algorithm defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  Blockio(std::function<void()> const &f) : func(f) { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Blockio(const Blockio& s) = delete; 

  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
    func();
  }

 private:
  std::function<void()> func;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * block instance to an output stream.\n
 * Does not print a newline control character.
 */
template < uint8_t N, uint8_t M, typename Tin = double, typename Tout = Tin >
std::ostream& operator<<(std::ostream& os, Blockio<N,M,Tin,Tout>& b) {
  os << "Generic block: '" << b.getName() << "'"; 
  return os;
}

}
}
#endif /* ORG_EEROS_CONTROL_BLOCKIO_HPP_ */
