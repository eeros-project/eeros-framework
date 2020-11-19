#ifndef ORG_EEROS_SEQUENCER_STEP_HPP_
#define ORG_EEROS_SEQUENCER_STEP_HPP_

#include <eeros/sequencer/BaseSequence.hpp>

namespace eeros {
namespace sequencer {

/**
 * A \ref Sequence comprises of several steps. A step is a single action.
 * To define your own step you have to extend this class and implement
 * your own \ref action method. You can also choose with which parameters
 * your step should be called.
 * 
 * @since v1.0
 */
class Step : public BaseSequence {
 public:
  /**
   * Constructs a step instance with a name and a reference to
   * the calling sequence.
   *
   * @param name - name of the step
   * @param caller - calling sequence
   */
  Step(std::string name, BaseSequence* caller) : BaseSequence(caller, true) {this->name = name;}
  
  /**
   * Disabling use of copy constructor because a Step should never be copied unintentionally.
   */
  Step(const Step& s) = delete; 

  /**
   * Operator for function calls. If you do not override this operator in a derived class, 
   * the step can be called without any parameter and will start upon calling.\n
   * You can add function call operators in a derived class with any number of parameters.
   */
  virtual int operator() () {return start();}
  
  /**
   * This is the function that is called when a step runs. This function
   * has to be implemented in the derived class. It can return a single 
   * value of type int. Make sure that this function does not wait or block.
   * 
   * @return - return value 
   */
  virtual int action() = 0;
 
 protected:
  int start() {
    resetTimeout();
    resetAbort();
    return BaseSequence::action();
  }
};

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_STEP_HPP_
