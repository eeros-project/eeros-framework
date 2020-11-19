#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <eeros/core/Fault.hpp>

#include <signal.h>
#include <chrono>
#include <gtest/gtest.h>

namespace seqTest4 {
using namespace eeros::sequencer;
int count = 0;
int eCount = 0;
int limit = 5;

class MyCondition : public Condition {
  bool validate() {return count >= 13 && eCount < 1;}  // just trigger once
};

class ExceptionSeq : public Sequence {
public:
  ExceptionSeq(std::string name, Sequence* caller) : Sequence(name, caller, true) { }
  int action() {
    count += 100;
    eCount++;
    if (eCount == 3) limit = 2;
    return 0;
  }
};

class MainSequence : public Sequence {
 public:
  MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), e1("e1", this), m("mon", this, cond, SequenceProp::abort, &e1) {
    addMonitor(&m);
  }  
  int action() {
    count = 10;
    return count;
  }
  bool checkExitCondition() {return count++ >= 121;}
  ExceptionSeq e1;
  MyCondition cond;
  Monitor m;
};

// Test condition
TEST(seqTest4, condition) {
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  MainSequence mainSeq("Main Sequence", sequencer);
  count = 0;
  eCount = 0;
  mainSeq.m.setBehavior(SequenceProp::abort);
  mainSeq();
  sequencer.wait();
  EXPECT_EQ(count, 114);
  EXPECT_EQ(eCount, 1);
}


}

