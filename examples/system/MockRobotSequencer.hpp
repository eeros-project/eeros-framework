#ifndef MOCK_ROBOT_SEQUENCER_HPP_
#define MOCK_ROBOT_SEQUENCER_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Wait.hpp>
#include "MockRobotControlSystem.hpp"
#include "MockRobotSafetyProperties.hpp"

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

class MoveUp : public Step {
 public:
  MoveUp(std::string name, Sequence* caller, MockRobotControlSystem& cs) : Step(name, caller), cs(cs) { }
  int action() {
    Matrix<2,1,double> dest{8.2, 27.0};
    cs.pp.move(dest);
    return 0;
  }
  bool checkExitCondition() {return cs.pp.endReached();}
  MockRobotControlSystem& cs;
};

class MoveDown : public Step {
 public:
  MoveDown(std::string name, Sequence* caller, MockRobotControlSystem& cs) : Step(name, caller), cs(cs) { }
  int action() {
    Matrix<2,1,double> dest{-5, 3};
    cs.pp.move(dest);
    return 0;
  }
  bool checkExitCondition() {return cs.pp.endReached();}
  MockRobotControlSystem& cs;
};

class UpAndDownSequence : public Sequence {
 public:
  UpAndDownSequence(std::string name, Sequence* caller, MockRobotControlSystem& cs) 
      : Sequence(name, caller, true), 
        moveUp("move up", this, cs), 
        moveDown("move down", this, cs) { }
  int action() {
    while (Sequencer::running) {
      moveUp();
      moveDown();
    }
    return 0;
  }
  MoveUp moveUp;
  MoveDown moveDown;
};

class HomingSequence : public Sequence {
 public:
  HomingSequence(std::string name, Sequence* caller, MockRobotControlSystem& cs, SafetySystem& ss, MockRobotSafetyProperties& sp) 
      : Sequence(name, caller, true), cs(cs), ss(ss), sp(sp) { }
  int action() {
    cs.setpoint.setValue({0.1, 0.1});
    return 0;
  }
  bool checkExitCondition() {
    Matrix<2,1,double> val = cs.i.getOut().getSignal().getValue();
    bool done = val[0] >= 1.0 && val[1] >= 1.0;
    if (val[0] >= 1.0) cs.setpoint.setValue({0, cs.setpoint.getValue()[1]});
    if (val[1] >= 1.0) cs.setpoint.setValue({cs.setpoint.getValue()[0], 0});
    if (done) ss.triggerEvent(sp.homingDone);
    return done;
  }
  MockRobotControlSystem& cs;
  SafetySystem& ss;
  MockRobotSafetyProperties& sp;
};


class MainSequence : public Sequence {
 public:
  MainSequence(std::string name, Sequencer& seq, MockRobotControlSystem& cs, SafetySystem& ss, MockRobotSafetyProperties& sp) 
      : Sequence(name, seq), cs(cs), ss(ss), sp(sp),
        homing("Homing Sequence", this, cs, ss, sp),  
        upDown("UpDown Sequence", this, cs),
        wait("wait", this) { }
  int action() {
    while(Sequencer::running) {
      if (ss.getCurrentLevel() == sp.slHoming) {
        homing();
      } else if(ss.getCurrentLevel() == sp.slReady) {
        wait(2);
        cs.sw.switchToInput(1);
        ss.triggerEvent(sp.startMoving);
      } else if(ss.getCurrentLevel() == sp.slMoving) {
        upDown();
      }
      wait(0.1);
    }
    return 0;
  }
  MockRobotControlSystem& cs;
  SafetySystem& ss;
  MockRobotSafetyProperties& sp;
  HomingSequence homing;
  UpAndDownSequence upDown;
  Wait wait;
};

#endif // MOCK_ROBOT_SEQUENCER_HPP_