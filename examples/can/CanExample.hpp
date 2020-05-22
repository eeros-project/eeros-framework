#include <iostream>
#include <signal.h>

#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Trace.hpp>

#include <eeros/control/can/CanHandle.hpp>
#include <eeros/control/can/CanReceiveFaulhaber.hpp>
#include <eeros/control/can/CanSendFaulhaber.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Wait.hpp>

using namespace eeros;
using namespace eeros::math;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::sequencer;

static constexpr uint8_t node1 = 1;           // id of node 1
static constexpr uint8_t node2 = 2;           // id of node 2
static constexpr uint8_t nofNodes = 2;        // nof nodes

static constexpr uint32_t warn_emergency = 0x00000001;
static constexpr uint32_t warn_posLimit  = 0x01000000;

class ControlSystem {
 public:
  ControlSystem(double ts) 
      : handle("can1"), 
        canReceive(handle.getSocket(), {node1, node2}, {CANOPEN_FC_PDO1_TX, CANOPEN_FC_PDO2_TX}),
        canSend(handle.getSocket(), {node1, node2}, {CANOPEN_FC_PDO1_RX, CANOPEN_FC_PDO2_RX}),
        timedomain("Main time domain", ts, true) {
    timedomain.addBlock(canReceive);        
    timedomain.addBlock(canSend);        
    Executor::instance().add(timedomain);
  }
    
  CanHandle handle;
  CanReceiveFaulhaber<nofNodes> canReceive;
  CanSendFaulhaber<nofNodes> canSend;
  TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
 public:
  MySafetyProperties() 
      : slInit("init"), slReady("ready"), slMoving("moving"), 
        seInitDone("init done"), seStartMoving("start moving") {    
    addLevel(slInit);
    addLevel(slReady);
    addLevel(slMoving);
    setEntryLevel(slInit);
  }
    
  SafetyLevel slInit, slReady, slMoving;
  SafetyEvent seInitDone, seStartMoving;
};

class WaitForState : public Step {
 public:
  WaitForState(std::string name, Sequence* caller, ControlSystem& cs) : Step(name, caller), cs(cs) { }
  int operator() (uint32_t state) {this->state = state; return start();}
  int action() {
    return 0;
  }
  bool checkExitCondition() {
    auto status = cs.canReceive.getOutStatus().getSignal().getValue();
    return ((status[0] & 0x6f) == state) && ((status[1] & 0x6f) == state);
  }
 private:
  uint32_t state;
  ControlSystem& cs;
};

class InitSequence : public Sequence {
 public:
  InitSequence(std::string name, Sequence* caller, ControlSystem& cs, SafetySystem& ss, MySafetyProperties& sp) 
      : Sequence(name, caller, true), cs(cs), ss(ss), sp(sp), wait("Wait", this), waitForState("Wait for state", this, cs) { }
        
  int action() {
    int socket = cs.handle.getSocket();
    uint32_t warningData1, warningData2;
        
    log.info() << "init can nodes";
    init_can_nodes(socket);  // nodes should be operational after this
    wait(0.5);
    // get warnings by SDO transfer
    get_warning_drive(socket, node1, &warningData1);
    get_warning_drive(socket, node2, &warningData2);
    if ((warningData1 == warn_emergency) || (warningData2 == warn_emergency)) {
      log.info() << "emergency button pressed (STO active)";
    } else if (((warningData1 == 0) || (warningData1 == warn_posLimit)) && ((warningData2 == 0) || (warningData2 == warn_posLimit))){
      //      if(initDrive.getNofActivations() == 1) {
      uint32_t status1, status2;
      get_status_drive(socket, node1, &status1);
      get_status_drive(socket, node2, &status2);
      log.info() << "drive 1 status = 0x" << std::hex << status1;
      log.info() << "drive 2 status = 0x" << std::hex << status2;
                
      set_profile_velocity_mode_faulhaber(socket, node1);
      set_profile_velocity_mode_faulhaber(socket, node2);

      // enable canSend- and canReceive-Block
      // from now on no SDO transfers possible
      cs.canSend.enable();
      cs.canReceive.enable();
    } else {
      log.info() << "warningData node 1: " << warningData1;
      log.info() << "warningData node 2: " << warningData2;
      return 0;
    }
    wait(0.5);
    auto status = cs.canReceive.getOutStatus().getSignal().getValue();
    log.info() << "drive status = triggerEvent(startInitDrive); 0x" << std::hex << status;
    cs.canSend.getInCtrl().getSignal().setValue(0x80); // send fault reset
    wait(0.5);
    log.info() << "drive status = 0x" << std::hex << cs.canReceive.getOutStatus().getSignal().getValue();
    cs.canSend.getInCtrl().getSignal().setValue(0); // release fault reset
    waitForState(0x40); // wait for state "Switch on disabled"
    log.info() << "drive status 'Switch on disabled'";
    cs.canSend.getInCtrl().getSignal().setValue(6);    // send shutdown, next state will be "Ready to switch on"
    waitForState(0x21); // wait for state "Ready to switch on"
    log.info() << "drive status 'Ready to switch on'";
    cs.canSend.getInCtrl().getSignal().setValue(7);    // send switch on, next state will be "Switch on"
    waitForState(0x23); // wait for state "Switch on"
    log.info() << "drive status 'Switch on'";

    wait(0.5);
    log.info() << "motors enabled";
    cs.canSend.ipmodeEnable();
//             controlSys->cCtrl.setValue(0x1f);    // send enable, next state will be "Operation enable"
            
    cs.canSend.getInCtrl().getSignal().setValue(0x1f);    // send enable

    ss.triggerEvent(sp.seInitDone);
    return 0;
  }
    
  bool checkExitCondition() {
    return true;
  }
    
 private:
  ControlSystem& cs;
  SafetySystem& ss;
  MySafetyProperties& sp;
  Wait wait;
  WaitForState waitForState;
};

class MoveSequence : public Sequence {
 public:
  MoveSequence(std::string name, Sequence* caller, ControlSystem& cs) 
      : Sequence(name, caller, true), cs(cs), wait("Wait", this) { }
        
  int action() {
    return 0;
  }
    
 private:
  ControlSystem& cs;
  Wait wait;
};

class MainSequence : public Sequence {
 public:
  MainSequence(std::string name, Sequencer& seq, ControlSystem& cs, SafetySystem& ss, MySafetyProperties& sp) 
      : Sequence(name, seq), ss(ss), sp(sp),
        initSeq("Initialization Sequence", this, cs, ss, sp),  
        move("Move Sequence", this, cs),
        wait("Wait", this) { }
     
  int action() {
    while(Sequencer::running) {
      if (ss.getCurrentLevel() == sp.slInit) {
        initSeq();
      } else if(ss.getCurrentLevel() == sp.slReady) {
        wait(2);
        ss.triggerEvent(sp.seStartMoving);
      } else if(ss.getCurrentLevel() == sp.slMoving) {
        move();
      }
      wait(0.1);
    }
    return 0;
  }
 private:
   SafetySystem& ss;
   MySafetyProperties& sp;
   InitSequence initSeq;
   MoveSequence move;
   Wait wait;
};
