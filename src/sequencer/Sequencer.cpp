#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/SequencerUI.hpp>
#include <eeros/core/Fault.hpp>

namespace eeros {
namespace sequencer {
  
std::atomic<bool> Sequencer::running(true);
int Sequencer::sequenceCount = 0;

Sequencer::Sequencer() : log(logger::Logger::getLogger('R')), stepping(false), nextStep(false) {
  running = true;
}

Sequencer::~Sequencer() { }

Sequencer& Sequencer::instance() {
  static Sequencer seq;
  return seq;
}

void Sequencer::addSequence(Sequence& seq) {
  seq.setId(sequenceCount++);
  sequenceList.push_back(&seq);
}

Sequence* Sequencer::getSequenceById(int id) {
  for (Sequence *seq : getListOfAllSequences()) {
    if (id == seq->getId()) return seq;
  }
  log.error() << "No sequence with id '" << id << "' found.";
  return nullptr;
}

Sequence* Sequencer::getSequenceByName(std::string name) {
  for (Sequence *seq : getListOfAllSequences()) {
    if ( name == seq->getName() ) return seq;
  }
  log.error() << "No sequence with name '" << name << "' found.";
  return nullptr;
}

std::vector<Sequence*> Sequencer::getListOfAllSequences() {
  return sequenceList;
}

void Sequencer::clearList() {
  sequenceList.clear();
  sequenceCount = 0;
}

void Sequencer::singleStepping() {
  stepping = true;
}

void Sequencer::step() {
  nextStep = true;
}

void Sequencer::restart() {
  stepping = false;
  nextStep = true;
}

void Sequencer::wait() {
  std::vector<Sequence*> list = getListOfAllSequences();
  for (Sequence* s : list) {
    s->wait();
  }
}

// can be used to terminate all sequences
void Sequencer::abort() {
  std::vector<Sequence*> list = getListOfAllSequences();
  for (Sequence* s : list) {
    s->conditionAbort.set();
  }
  running = false;
}

} // namespace sequencer
} // namespace eeros
























