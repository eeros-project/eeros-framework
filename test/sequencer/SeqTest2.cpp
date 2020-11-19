#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <eeros/core/Fault.hpp>

#include <signal.h>
#include <chrono>
#include <gtest/gtest.h>

namespace seqTest2 {
using namespace eeros::sequencer;

class Step1 : public Step {
 public:
  Step1(std::string name, BaseSequence* caller) : Step(name, caller) { }
  int action() {count++; return count;}
 private:
  int count = 0;
};

// class Step2 : public Step {
//  public:
//   Step2(std::string name, BaseSequence* caller) : Step(name, caller) { }
//   bool checkPreCondition() {return count == 10);
//   int action() {count++; return count;}
//   int count = 0;
// };

class MainSequence : public Sequence {
 public:
  MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), step("step", this) { }
    
  int action() {
    int count = 0;
    for (int i = 0; i < 5; i++) {
      count = step();
      if (count == 3) kill(getpid(), SIGINT);
    }
    return count;
  }
 private:
  Step1 step;
};

class SequenceA : public Sequence {
 public:
  SequenceA(std::string name, Sequencer& seq) : Sequence(name, seq) { }
  bool checkPreCondition() {return count == 5;};
  int action() {
    count++;
    return count;
  }
  int count = 0;
};

class SequenceB : public Sequence {
public:
  SequenceB(std::string name, Sequencer& seq) : Sequence(name, seq) { }
  int action() {
    count++;
    return count;
  }
  bool checkExitCondition() {return count == 15;}
  int count = 0;
};

void signalHandler(int signum) {
  Sequencer::instance().abort();
}

// Test aborting
TEST(seqTest2, aborting) {
  signal(SIGINT, signalHandler);
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  MainSequence mainSeq("Main Sequence", sequencer);
  mainSeq();
  sequencer.wait();
  EXPECT_EQ(mainSeq.getResult(), -1);
}

// check preCondition
TEST(seqTest2, preCondition) {
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  SequenceA s("s", sequencer);
  s();
  sequencer.wait();
  EXPECT_EQ(s.getResult(), -1);
  EXPECT_EQ(s.count, 0);
  s.count = 5;
  s();
  sequencer.wait();
  EXPECT_EQ(s.getResult(), 6);
  EXPECT_EQ(s.count, 6);
}

// check exitCondition
TEST(seqTest2, exitCondition) {
  auto& sequencer = Sequencer::instance();
  sequencer.clearList();
  SequenceB s("s", sequencer);
  s();
  usleep(1000);
  s.count = 15;
  sequencer.wait();
  EXPECT_EQ(s.getResult(), 1);
  EXPECT_EQ(s.count, 15);
}


}