#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <eeros/core/Fault.hpp>

#include <signal.h>
#include <chrono>
#include <gtest/gtest.h>

namespace seqTest1 {
using namespace eeros::sequencer;

class Step1 : public Step {
 public:
  Step1(std::string name, BaseSequence* caller) : Step(name, caller) { }
  int operator()() {return start();}
  int operator()(int val) {count += val; return start();}
  int action() {count++; return count;}
  int count = 0;
};

class MainSequence : public Sequence {
 public:
  MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), step("step", this) { }
  int action() {
    int count = 0;
    for (int i = 0; i < 5; i++) {
      count = step();
    }
    return count;
  }
  Step1 step;
};

class SequenceA : public Sequence {
 public:
  SequenceA(std::string name, Sequencer& seq, Sequence* caller) : Sequence(name, caller, true), step("step", this) { }
  SequenceA(std::string name, Sequencer& seq) : Sequence(name, seq), step("step", this) { }
  int operator()() {return start();}
  int operator()(int val) {count = val; return start();}
  int action() {
    return step(count);
  }
  Step1 step;
  int count = 100;
};

class SequenceB : public Sequence {
 public:
  SequenceB(std::string name, Sequencer& seq, Sequence* caller) : Sequence(name, caller, false) { }
  int action() {
    return 20;
  }
};


// Test stepping and return values
TEST(seqTest1, stepping) {
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  MainSequence mainSeq("Main Sequence", sequencer);
  mainSeq();
  sequencer.wait();
  EXPECT_EQ(mainSeq.getResult(), 5);
}

// Test registration
TEST(seqTest1, list) {
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  EXPECT_EQ(sequencer.getListOfAllSequences().size(), 0);
  try {
    MainSequence mainSeq("", sequencer);
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("all sequences must have a name"));
  }
  
  sequencer.clearList();
  MainSequence mainSeq("Main Sequence", sequencer);
  EXPECT_EQ(sequencer.getListOfAllSequences().size(), 1);
  Sequence* s1 = sequencer.getSequenceByName("Main");
  EXPECT_EQ(s1, nullptr);
  s1 = sequencer.getSequenceByName("Main Sequence");
  EXPECT_EQ(s1, &mainSeq);

  try {
    MainSequence mainSeq1("Main Sequence", sequencer);
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("all sequences must have different names"));
  }
  EXPECT_EQ(sequencer.getListOfAllSequences().size(), 1);
  MainSequence mainSeq1("Main Sequence1", sequencer);
  EXPECT_EQ(sequencer.getListOfAllSequences().size(), 2);
 
  EXPECT_EQ(sequencer.getSequenceById(0), &mainSeq);
  EXPECT_EQ(sequencer.getSequenceById(1), &mainSeq1);
}

// Test waiting
TEST(seqTest1, waiting) {
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  MainSequence mainSeq("Main Sequence", sequencer);
  SequenceA seqA("seqA", sequencer, &mainSeq);
  SequenceB seqB("seqB", sequencer, &mainSeq);
  EXPECT_EQ(sequencer.getListOfAllSequences().size(), 3);
  EXPECT_EQ(sequencer.getSequenceById(0), &mainSeq);
  EXPECT_EQ(sequencer.getSequenceById(1), &seqA);  
  EXPECT_EQ(sequencer.getSequenceById(2), &seqB);  
  mainSeq();
  sequencer.wait();
  EXPECT_EQ(mainSeq.getResult(), 5);
}

// Test parameters
TEST(seqTest1, params) {
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  SequenceA seqA("seqA", sequencer);
  seqA();
  sequencer.wait();
  EXPECT_EQ(seqA.getResult(), 101);
  EXPECT_EQ(seqA.step.count, 101);
  EXPECT_EQ(seqA.count, 100);
  seqA(1000);
  sequencer.wait();
  EXPECT_EQ(seqA.getResult(), 1102);
  EXPECT_EQ(seqA.step.count, 1102);
  EXPECT_EQ(seqA.count, 1000);
}

}

