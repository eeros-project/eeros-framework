#ifndef ORG_EEROS_CONTROL_BLOCKIO_HPP_
#define ORG_EEROS_CONTROL_BLOCKIO_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>
#include <functional>
#include <array>
#include <ostream>

namespace eeros::control {

template<uint8_t N> concept HasInputs  = (N > 0);
template<uint8_t M> concept HasOutputs = (M > 0);

/**
 * Base class for all blocks with inputs and outputs.
 *
 * Extend this class and override @ref run() to implement any algorithm.
 * Alternatively, pass a lambda directly to the constructor for simple
 * inline algorithms without subclassing:
 *
 * @code
 * Blockio<2,1,Vector2,Vector2> block([&]() {
 *   auto val = (block.getIn(0).getSignal().getValue() + 0.5) * 2;
 *   val[0] *= -1.0;
 *   val += block.getIn(1).getSignal().getValue() + 1.0;
 *   block.getOut().getSignal().setValue(val);
 *   block.getOut().getSignal().setTimestamp(gen.getIn(0).getSignal().getTimestamp());
 * });
 * @endcode
 *
 * @tparam N    Number of inputs
 * @tparam M    Number of outputs
 * @tparam Tin  Input signal type (default: @c double)
 * @tparam Tout Output signal type (default: same as @c Tin)
 *
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
  explicit Blockio(std::function<void()> const& f) : func(f) {
    if constexpr (HasInputs<N>)
      for (uint8_t i = 0; i < N; ++i) in[i].setOwner(this);
    if constexpr (HasOutputs<M>)
      for (uint8_t i = 0; i < M; ++i) {
        out[i].setOwner(this);
        out[i].getSignal().clear();
      }
  }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  Blockio(const Blockio&) = delete;
  Blockio& operator=(const Blockio&) = delete;

  /**
   * Runs the generic algorithm.
   *
   */
  void run() override { func(); }

  /**
   * @brief Returns input at @p index.
   * @throws IndexOutOfBoundsFault if @p index >= N
   */
  Input<Tin>& getIn(uint8_t index) requires HasInputs<N> {
    if (index >= N)
      throw IndexOutOfBoundsFault(
        "Trying to get inexistent element of input vector in block '" +
        this->getName() + "'");
    return in[index];
  }

  /**
   * @brief Returns input (single-input convenience overload, only when N == 1).
   */
  Input<Tin>& getIn() requires (N == 1) { return in[0]; }

  /**
   * @brief Returns output at @p index.
   * @throws IndexOutOfBoundsFault if @p index >= M
   */
  Output<Tout>& getOut(uint8_t index) requires HasOutputs<M> {
    if (index >= M)
      throw IndexOutOfBoundsFault(
        "Trying to get inexistent element of output vector in block '" +
        this->getName() + "'");
    return out[index];
  }

  /**
   * @brief Returns output (single-output convenience overload, only when M == 1).
   */
  Output<Tout>& getOut() requires (M == 1) { return out[0]; }

 protected:
  std::array<Input<Tin>, N> in;
  std::array<Output<Tout>,M> out;

 private:
  std::function<void()> func;
};


/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * block instance to an output stream.\n
 * Does not print a newline control character.
 */
template < uint8_t N, uint8_t M, typename Tin = double, typename Tout = Tin >
std::ostream& operator<<(std::ostream& os, const Blockio<N,M,Tin,Tout>& b) {
  os << "Generic block: '" << b.getName() << "'"; 
  return os;
}

}

#endif /* ORG_EEROS_CONTROL_BLOCKIO_HPP_ */
