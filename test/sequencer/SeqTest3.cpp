#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <eeros/core/Fault.hpp>

#include <signal.h>
#include <chrono>
#include <gtest/gtest.h>

namespace seqTest3 {
using namespace eeros::sequencer;
int count = 0;
int eCount = 0;
int limit = 5;

class Step1 : public Step {
 public:
  Step1(std::string name, BaseSequence* caller) : Step(name, caller) { }
  int action() {count++; return count;}
};

class MyCondition : public Condition {
  bool validate() {return count > 2 && count < 100;}
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
  MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), step("step", this), e1("e1", this), m("mon", this, cond, SequenceProp::abort, &e1) {
    addMonitor(&m);
  }  
  int action() {
    count = 0;
    for (int i = 0; i < limit; i++) {
      step();
    }
    return count;
  }
  Step1 step;
  ExceptionSeq e1;
  MyCondition cond;
  Monitor m;
};

class SequenceA : public Sequence {
 public:
  SequenceA(std::string name, Sequencer& seq, Sequence* caller) : Sequence(name, caller, true) { }
  int action() {
    return 10;
  }
 
};

class SequenceB : public Sequence {
 public:
  SequenceB(std::string name, Sequencer& seq, Sequence* caller) : Sequence(name, caller, false) { }
  int action() {
    return 20;
  }
};


// Test condition
TEST(seqTest3, condition) {
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  MainSequence mainSeq("Main Sequence", sequencer);
  count = 0;
  eCount = 0;
  mainSeq.m.setBehavior(SequenceProp::abort);
  mainSeq();
  sequencer.wait();
  EXPECT_EQ(count, 105);
  EXPECT_EQ(eCount, 1);
  count = 0;
  eCount = 0;
  mainSeq.m.setBehavior(SequenceProp::resume);
  mainSeq();
  sequencer.wait();
  EXPECT_EQ(count, 105);
  EXPECT_EQ(eCount, 1);
  count = 0;
  eCount = 0;
  mainSeq.m.setBehavior(SequenceProp::restart);
  mainSeq();
  sequencer.wait();
  EXPECT_EQ(count, 2);
  EXPECT_EQ(eCount, 3);
}


}

