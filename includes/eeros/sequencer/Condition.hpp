#ifndef ORG_EEROS_SEQUENCER_CONDITION_HPP_
#define ORG_EEROS_SEQUENCER_CONDITION_HPP_

namespace eeros {
namespace sequencer {

/**
 * A condition is checked by a \ref Monitor. You can define your 
 * own conditions by extending this class.
 * 
 * @since v1.0
 */
class Condition {
  friend class Monitor;
 public:
  /**
   * This is the function that is called by a \ref Monitor when it
   * checks the condition. This function has to be implemented in the derived class. 
   * It must return true when the condition is met.
   * 
   * @return - return true if condition is met 
   */
  virtual bool validate() = 0;

 private:
  bool isTrue() {return validate();}
};

} // namespace sequencer
} // namespace eeros

#endif //ORG_EEROS_SEQUENCER_CONDITION_HPP_