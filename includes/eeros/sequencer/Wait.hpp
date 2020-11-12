#ifndef ORG_EEROS_SEQUENCER_WAIT_HPP_
#define ORG_EEROS_SEQUENCER_WAIT_HPP_

#include <eeros/sequencer/Step.hpp>

namespace eeros {
namespace sequencer {

/**
 * This is a special \ref step which simply wait for a given time.
 * 
 * @since v1.0
 */
class Wait : public Step {
public:
  /**
   * Constructs a wait step instance with a name and a reference to
   * the calling sequence.
   *
   * @param name - name of the step
   * @param caller - calling sequence
   */
  Wait(std::string name, BaseSequence* caller) : Step(name, caller) { }

  /**
   * Disabling use of copy constructor because a Step should never be copied unintentionally.
   */
  Wait(const Wait& s) = delete; 

  /**
   * Destructor
   */  
  virtual ~Wait() { };
  
  /**
   * Operator for function calls. Sets the waiting time and starts waiting.
   * 
   * @param waitingTime - waiting time in sec
   */
  int operator() (double waitingTime) {this->waitingTime = waitingTime; return start();}
 
 private:
  int action() {time = std::chrono::steady_clock::now(); return 0;}
  bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > waitingTime;}
  
  std::chrono::time_point<std::chrono::steady_clock> time;
  double waitingTime;
};

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_WAIT_HPP_
