#ifndef ORG_EEROS_CONTROL_BLOCKIO_HPP_
#define ORG_EEROS_CONTROL_BLOCKIO_HPP_

#include <algorithm>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>
#include <functional>

namespace eeros {
namespace control {

template<uint8_t N>
concept None = (N == 0);

template<uint8_t N>
concept One = (N == 1);

template<uint8_t N>
concept Multiple = (N > 1);

namespace
{
  template <class F, std::size_t... Is>
  void for_(F func, std::index_sequence<Is...>)
  {
    (func.template operator()<Is>(), ...);
  }
}

/**
 * Helper that generates the requires n-sized SIUnit std::array from a SIUnit.
 * Can be used for Blockio implementations that want to create an array containing a single SIUnit or an array of multiple SIUnits with the same value,
 * by forwarding their received template SIUnit argument into this helper struct and accessing the created array with ::value.
 * 
 * Used by calling MakeUnitArray<U, N>::value in the template paramter list to Blockio, where U is the SIUnit and N the size.
 * 
 * @tparam U Unit that should be used to create an std::array containing that element.
 * @tparam N How many times the Unit sould be inserted, default = 1.
 */
template<SIUnit U, std::size_t N = 1>
struct MakeUnitArray {
  static constexpr std::array<SIUnit, N> value = siunit::generateNSizeArray<N, U>();
};

/**
 * Helper that allow to generate a unrolled for loop at compile time.
 * Necessary to be able to pass the indecies into the for loop as template arguments to still be able to call getIn<N>() and getOut<N>().
 * 
 * @tparam N Number that should be transformed into indicies for the for loop from 0 to N
 * @tparam F Functor that receives the transformed indicies from 0 to N
 * @param func Functor that should be executed
 */
template <std::size_t N, typename F>
void for_(F func)
{
  for_(func, std::make_index_sequence<N>());
}

struct Empty {};

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
 * @tparam Tin - input signal data type (double - default type)
 * @tparam Tout - output signal data type (double - default type)
 * @tparam Uin - input signal unit types (dimensionless - default type)
 * @tparam Uout - output signal unit types (dimensionless - default type)
 * @since v1.2.1
 */

template < uint8_t N, uint8_t M, typename Tin = double, typename Tout = Tin, std::array<SIUnit, N> Uin = siunit::generateNSizeArray<N>(), std::array<SIUnit, M> Uout = siunit::generateNSizeArray<M>() >
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
  Blockio(std::function<void()> const &f) : func(f), in(generateNInputs()), out(generateMOutputs()) {
    initalizeInputsAndOutputs();
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
   * Get an output of the multiple instances of the block.
   * With additional compile time safety if the passed index is out of scope.
   * 
   * @note Callable only if the instance has multiple inputs N > 1.
   * 
   * @tparam I - compile time constant index of the input
   * @return input
   */
  template<size_t I>
  decltype(auto) getIn() requires Multiple<N> {
    return std::get<I>(in);
  }

  /**
   * Get an output of the multiple instances of the block.
   * 
   * @note Callable only if the instance has multiple Inputs N > 1 and the SIUnit instances of those Inputs are all dimensionless.
   * 
   * @param index - runtime index of the input
   * @return input
   */
  decltype(auto) getIn(uint8_t index) requires Multiple<N> {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return in[index];
  }

  /**
   * Get the single input of the block.
   * 
   * @note Callable only if the instance has one input N == 1.
   * 
   * @return output
   */
  auto& getIn() requires One<N> {
    return in;
  }

  /**
   * Get an output of the multiple instances of the block.
   * With additional compile time safety if the passed index is out of scope.
   * 
   * @note Callable only if the instance has multiple Outputs M > 1.
   * 
   * @tparam I - compile time constant index of the output.
   * @return output
   */
  template<size_t I>
  decltype(auto) getOut() requires Multiple<M> {
    return std::get<I>(out);
  }

  /**
   * Get an output of the multiple instances of the block.
   * 
   * @note Callable only if the instance has multiple Outputs M > 1 and the SIUnit instances of those Outputs are all dimensionless.
   * 
   * @param index - runtime index of the output
   * @return output
   */
  decltype(auto) getOut(uint8_t index) requires Multiple<M> {
    if (index >= M) throw IndexOutOfBoundsFault("Trying to get inexistent element of output vector in block '" + this->getName() + "'"); 
    return out[index];
  }

  /**
   * Get the single output of the block.
   * 
   * @note Callable only if the instance has one output, M == 1.
   * 
   * @return output
   */
  auto& getOut() requires One<M> {
    return out;
  }

 private:
  std::function<void()> func;

  /**
   * Initalizes the inputs and outputs by setting the owner to this instance and by additonally clearing the internal signal of outputs.
   */
  constexpr void initalizeInputsAndOutputs() {
    if constexpr (One<M>) {
      out.setOwner(this);
      out.getSignal().clear();
    }
    else if constexpr (Multiple<M>) {
      std::apply([this](auto&&... out) {
        ((out.setOwner(this)), ...);
        ((out.getSignal().clear()), ...);
      }, out);
    }

    if constexpr (One<N>) {
      in.setOwner(this);
    }
    else if constexpr (Multiple<N>) {
      std::apply([this](auto&&... in) {
        ((in.setOwner(this)), ...);
      }, in);
    }
  }

  /**
   * @brief Create the type that holds the inputs and instantiate it, by combining the passed template parameters.
   * 
   * @tparam Is Template parameter pack of a sequence from 0 - N, used to create the tuple type.
   * @return inputs
   */
  template<std::size_t... Is>
  constexpr static decltype(auto) createInputs(std::index_sequence<Is...>) {
    constexpr bool allSameDimension = std::ranges::all_of(Uin, [](auto e) { return e == Uin.at(0U); });
    if constexpr (allSameDimension) {
      return std::array<Input<Tin, Uin.at(0U)>, N>{};
    }
    else {
      return std::tuple<Input<Tin, Uin[Is]>...>{};
    }
  }

  /**
   * @brief Generate the n inputs requested, handling the different edge cases of a value of 0, 1 or N.
   * 
   * @return inputs
   */
  constexpr static decltype(auto) generateNInputs() {
    if constexpr (One<N>) {
      return Input<Tin, Uin[0U]>{};
    }
    else if constexpr (None<N>) {
      return Empty{};
    }
    else {
      return createInputs(std::make_index_sequence<N>{});
    }
  }

  /**
   * @brief Create the type that holds the outputs and instantiate it, by combining the passed template parameters.
   * 
   * @tparam Is Template parameter pack of a sequence from 0 - M, used to create the tuple type.
   * @return outputs
   */
  template<std::size_t... Is>
  constexpr static decltype(auto) createOutputs(std::index_sequence<Is...>) { 
    constexpr bool allSameDimension = std::ranges::all_of(Uout, [](auto e) { return e == Uout.at(0U); });
    if constexpr (allSameDimension) {
      return std::array<Output<Tout, Uout.at(0U)>, M>{};
    }
    else {
      return std::tuple<Output<Tout, Uout[Is]>...>{};
    }
  }

  /**
   * @brief Generate the m outputs requested, handling the different edge cases of a value of 0, 1 or M.
   * 
   * @return inputs
   */
  constexpr static decltype(auto) generateMOutputs() {
    if constexpr (One<M>) {
      return Output<Tout, Uout[0U]>{};
    }
    else if constexpr (None<M>) {
      return Empty{};
    }
    else {
      return createOutputs(std::make_index_sequence<M>{});
    }
  }

 protected:
  decltype(generateNInputs()) in;
  decltype(generateMOutputs()) out;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * block instance to an output stream.\n
 * Does not print a newline control character.
 */
template < uint8_t N, uint8_t M, typename Tin = double, typename Tout = Tin, std::array<SIUnit, static_cast<std::size_t>(N)> Uin = siunit::generateNSizeArray<N>(), std::array<SIUnit, static_cast<std::size_t>(M)> Uout = siunit::generateNSizeArray<M>() >
std::ostream& operator<<(std::ostream& os, Blockio<N,M,Tin,Tout, Uin, Uout>& b) {
  os << "Generic block: '" << b.getName() << "'"; 
  return os;
}

}
}
#endif /* ORG_EEROS_CONTROL_BLOCKIO_HPP_ */
