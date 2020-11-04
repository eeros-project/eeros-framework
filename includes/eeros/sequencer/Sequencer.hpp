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
  void singleStepping();
  static bool running;

 private:
  Sequencer();
  void addSequence(Sequence& seq);
  void step();
  void restart();
  Sequence* mainSequence;
  std::vector<Sequence*> sequenceList;	// list of all sequences
  eeros::logger::Logger log;	
  unsigned int id;
  bool stepping;
  volatile bool nextStep;
  SequencerUI ui;
  static int sequenceCount;
};

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
