#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <eeros/sequencer/BaseSequence.hpp>
#include <condition_variable>
#include <future>

namespace eeros {
namespace sequencer {

class Sequencer;

/**
 * A \ref Sequence comprises of several steps defined in its action function.
 * To define your own sequence you have to extend this class and implement
 * your own \ref action method. You can also choose with which parameters
 * your sequence should be called.
 * 
 * @since v1.0
 */
class Sequence : public BaseSequence {
  friend class Sequencer;
 public:
  /**
   * Constructs a main sequence instance with a name and a reference to
   * the sequencer which handles all sequences. Use this only for main sequences! 
   * Main sequences are by definition non blocking, that is, they run concurrently
   * to the calling thread. 
   * 
   * @see Sequence(std::string name, BaseSequence* caller, bool blocking)
   *
   * @param name - name of the step
   * @param seq - sequencer which handles this sequence
   */
  Sequence(std::string name, Sequencer& seq);	// only for mainSequence

  /**
   * Constructs a sequence instance with a name and a reference to
   * the calling sequence. You can also choose whether it should block or not. 
   * A blocking sequence blocks its calling sequence so that no more steps in the calling 
   * sequence will execute as long as this sequence runs. A non blocking sequence starts
   * a new thread and runs in parallel to the calling sequence.
   * 
   * @param name - name of the step
   * @param caller - calling sequence
   * @param blocking - will be blocking if true
   */
  Sequence(std::string name, BaseSequence* caller, bool blocking);
  
  /**
   * Disabling use of copy constructor because a sequence should never be copied unintentionally.
   */
  Sequence(const Sequence& s) = delete; 
  
  /**
   * Operator for function calls. If you do not override this operator in a derived class, 
   * the sequence can be called without any parameter and will start upon calling.\n
   * You can add function call operators in a derived class with any number of parameters.
   */
  virtual int operator() () {return start();}
  
  /**
   * This is the function that is called when a sequence runs. This function
   * has to be implemented in the derived class. It can return a single 
   * value of type int. Make sure that this function does not wait or block.
   * 
   * @return - return value 
   */
  virtual int action() = 0;
  
  /**
   * Waits for this sequence to finish its current run. This call will block.
   */
  void wait();

  /**
   * If a caller of a non blocking sequence wants to read its return value, it
   * must first wait (@see wait()) and then get the result with this call.
   * 
   * @return - return value 
   */ 
  int getResult();

  /**
   * Check if sequence has finished running
   *
   * @return true if done, false otherwise
   */
  bool done();
 
 protected:
  int start();
 
 private:
  Sequence(std::string name, Sequencer& seq, BaseSequence* caller, bool blocking);
  int run();
  std::future<int> fut;
  int retVal;
};

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
