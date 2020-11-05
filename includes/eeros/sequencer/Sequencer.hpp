#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#include <eeros/sequencer/SequencerUI.hpp>
#include <eeros/logger/Logger.hpp>
#include <vector>
#include <atomic>
#include <memory>

namespace eeros {
namespace sequencer {

class Sequence;
class SequencerUI;

class Sequencer {
  friend class BaseSequence;
  friend class Sequence;
  friend class SequencerUI;
  
 public:
  virtual ~Sequencer();
  static Sequencer& instance();
  
  Sequence* getSequenceById(int id);
  Sequence* getSequenceByName(std::string name);
  std::vector<Sequence*> getListOfAllSequences();
  
  /**
   * Clears the list with all sequences
   */
  void clearList();
  
  /**
   * Waits for the sequencer to terminate all its running sequences
   * (either blocking or nonblocking)
   */
  void wait();
  void abort();
  
  /**
   * The sequencer can be put into single stepping mode. This can be useful for
   * debugging purposes, E.g. upon entering a given sequence. 
   */
  void singleStepping();
  
  /**
   * State of the sequencer, set to true upon creation. Aborting the sequencer will 
   * set this variable to false and will abort all registered sequences.
   */
  static std::atomic<bool> running;

 private:
  Sequencer();
  void addSequence(Sequence& seq);
  void step();
  void restart();
  Sequence* mainSequence;
  std::vector<Sequence*> sequenceList;	// list of all sequences
  eeros::logger::Logger log;	
  std::atomic<bool> stepping;
  volatile std::atomic<bool> nextStep;
  SequencerUI ui;
  static int sequenceCount;
};

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
