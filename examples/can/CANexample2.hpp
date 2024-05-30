#include <eeros/control/can/CANopenReceive.hpp>
#include <eeros/control/can/CANopenSend.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <CANopen.hpp>
#include <DS402.hpp>

using namespace eeros;
using namespace eeros::math;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::sequencer;

static constexpr uint8_t nodeId1 = 1;           // id of node 1
static constexpr uint8_t nodeId2 = 2;           // id of node 2
static constexpr uint8_t nofNodes = 2;          // nof nodes

class ControlSystem {
 public:
  ControlSystem(CANopen& co, double ts)
      : co(co),
        canReceive(co, {nodeId1, nodeId2}),
        canSend(co, {nodeId1, nodeId2}),
        timedomain("Main time domain", ts, true) {
    velDrive0.setValue(0);
    velDrive1.setValue(0);
    canReceive.setName("CAN receive");
    canSend.setName("CAN send");
    canReceive.configureTPDO(nodeId1, TPDO1, {statusObj,posActValObj,digInStateObj}, {0,1,-1});
    canReceive.configureTPDO(nodeId2, TPDO1, {statusObj,posActValObj}, {0,1});
    canSend.configureRPDO(nodeId1, RPDO1, {controlObj,velModeSetValObj}, {0,1});
    canSend.configureRPDO(nodeId2, RPDO1, {controlObj,velModeSetValObj}, {0,1});

    Matrix<2,1,double> scale({50, 200});    // accounts for encoder resolution and gear box
    canReceive.setScale(scale);
    canSend.setScale(scale);
    canSend.setCtrl(0, 0xf);
    canSend.setCtrl(1, 0xf);
    canSend.getIn(0).connect(velDrive0.getOut());
    canSend.getIn(1).connect(velDrive1.getOut());
    timedomain.addBlock(canReceive);
    timedomain.addBlock(velDrive0);
    timedomain.addBlock(velDrive1);
    timedomain.addBlock(canSend);
    Executor::instance().add(timedomain);
  }
    
  CANopen& co;
  CANopenReceive<nofNodes> canReceive;
  CANopenSend<nofNodes,1> canSend;
  Constant<Matrix<1,1,double>> velDrive0, velDrive1;
  TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
 public:
  MySafetyProperties() 
      : slInit("init"), slReady("ready"), slMoving("moving"), slStop("stopping"),
        seInitDone("init done"), seStartMoving("start moving"), seShutDown("shutting down") {    
    addLevel(slInit);
    addLevel(slReady);
    addLevel(slMoving);
    addLevel(slStop);

    slInit.addEvent(seInitDone, slReady, kPublicEvent);
    slReady.addEvent(seStartMoving, slMoving, kPublicEvent);
    slReady.addEvent(seShutDown, slStop, kPublicEvent);
    slMoving.addEvent(seShutDown, slStop, kPublicEvent);
    setEntryLevel(slInit);
    
    exitFunction = [&](SafetyContext* privateContext) {    
      privateContext->triggerEvent(seShutDown);
    };
  }
    
  SafetyLevel slInit, slReady, slMoving, slStop;
  SafetyEvent seInitDone, seStartMoving, seShutDown;
};

class InitSequence : public Sequence {
 public:
  InitSequence(std::string name, Sequence* caller, ControlSystem& cs, SafetySystem& ss, MySafetyProperties& sp, DS402& ds402)
      : Sequence(name, caller, true), cs(cs), ss(ss), sp(sp), ds402(ds402), wait("Wait", this) { }
        
  int action() {
    ds402.co.sendNMT(0, ds402.co.NMT_CS::PREOP); // set all nodes to CS_PREOP

    ds402.configureTPDO(nodeId1, 1, {statusObj,posActValObj,digInStateObj});
    ds402.configureTPDO(nodeId1, 2, {});
    ds402.configureTPDO(nodeId1, 3, {});
    ds402.configureTPDO(nodeId1, 4, {});
    ds402.configureTPDO(nodeId2, 1, {statusObj,posActValObj});
    ds402.configureTPDO(nodeId2, 2, {});
    ds402.configureTPDO(nodeId2, 3, {});
    ds402.configureTPDO(nodeId2, 4, {});
    ds402.configureRPDO(nodeId1, 1, {controlObj,velModeSetValObj});
    ds402.configureRPDO(nodeId1, 2, {});
    ds402.configureRPDO(nodeId1, 3, {});
    ds402.configureRPDO(nodeId1, 4, {});
    ds402.configureRPDO(nodeId2, 1, {controlObj,velModeSetValObj});
    ds402.configureRPDO(nodeId2, 2, {});
    ds402.configureRPDO(nodeId2, 3, {});
    ds402.configureRPDO(nodeId2, 4, {});

    ds402.setOperationMode(nodeId1, ds402.OPERATION_MODES::VELOCITY);
    ds402.enableOperation(nodeId1);
    ds402.setOperationMode(nodeId2, ds402.OPERATION_MODES::VELOCITY);
    ds402.enableOperation(nodeId2);

    ds402.co.sendNMT(0, ds402.co.NMT_CS::START);

    // enable canSend and canReceive block, from now on no SDO transfers possible
    cs.canSend.enable();
    cs.canReceive.enable();

    log.info() << "drive 1: state = " << ds402.statusToString(ds402.getStatus(nodeId1));
    log.info() << "drive 1: error state = " << ds402.getErrorDesc(nodeId1);
    log.info() << "drive 2: state = " << ds402.statusToString(ds402.getStatus(nodeId2));
    log.info() << "drive 2: error state = " << ds402.getErrorDesc(nodeId2);

    ss.triggerEvent(sp.seInitDone);
    return 0;
  }
    
 private:
  ControlSystem& cs;
  SafetySystem& ss;
  MySafetyProperties& sp;
  DS402& ds402;
  Wait wait;
};

class MoveSequence : public Sequence {
 public:
  MoveSequence(std::string name, Sequence* caller, ControlSystem& cs, SafetySystem& ss, MySafetyProperties& sp, DS402& ds402)
      : Sequence(name, caller, true), cs(cs), ss(ss), sp(sp), ds402(ds402), wait("Wait", this), log(Logger::getLogger()) { }
        
  int action() {
    uint32_t vel = 2;
    while(Sequencer::running  && ss.getCurrentLevel() == sp.slMoving) {
      cs.velDrive0.setValue(vel);
      cs.velDrive1.setValue(vel);
      wait(2.0);
      log.info() << "act pos 1: " << cs.canReceive.getOut(0).getSignal().getValue() << " act pos 2: " << cs.canReceive.getOut(1).getSignal().getValue() << " dig 1: 0x" << hex << cs.canReceive.getDigOut(0).getSignal().getValue();
      log.info() << hex << "status 1: " << ds402.statusToString(cs.canReceive.getStatus(0)) << " status 2: " << ds402.statusToString(cs.canReceive.getStatus(1));
      if (vel == 2) vel = -1;
      else vel = 2;
    }
    return 0;
  }
    
 private:
  ControlSystem& cs;
  SafetySystem& ss;
  MySafetyProperties& sp;
  DS402& ds402;
  Wait wait;
  Logger log;
};

class StopSequence : public Sequence {
 public:
  StopSequence(std::string name, Sequence* caller, ControlSystem& cs, CANopen& co)
      : Sequence(name, caller, true), cs(cs), co(co), wait("Wait", this) { }
        
  int action() {
    cs.canReceive.disable();
    cs.canSend.disable();
    co.sendNMT(0, co.NMT_CS::PREOP); // set all nodes to CS_PREOP
    DS402 ds402(co);
    ds402.disableOperation(nodeId1);
    ds402.disableOperation(nodeId2);
    log.info() << "state of drive = " << ds402.statusToString(ds402.getStatus(nodeId1));
    log.info() << "state of drive = " << ds402.statusToString(ds402.getStatus(nodeId2));
    wait(0.5);
    Sequencer::instance().abort();
    Executor::instance().stop();
    return 0;
  }
    
 private:
  ControlSystem& cs;
  CANopen& co;
  Wait wait;
};

class MainSequence : public Sequence {
 public:
  MainSequence(std::string name, Sequencer& seq, ControlSystem& cs, SafetySystem& ss, MySafetyProperties& sp, CANopen& co)
      : Sequence(name, seq), ss(ss), sp(sp), ds402(co),
        init("Initialization Sequence", this, cs, ss, sp, ds402),
        move("Move Sequence", this, cs, ss, sp, ds402),
        stop("Stopping Sequence", this, cs, ds402.co),
        wait("Wait", this) { }
     
  int action() {
    while(Sequencer::running) {
      if (ss.getCurrentLevel() == sp.slInit) {
        init();
      } else if(ss.getCurrentLevel() == sp.slReady) {
        wait(2);
        ss.triggerEvent(sp.seStartMoving);
      } else if(ss.getCurrentLevel() == sp.slMoving) {
        move();
      } else if(ss.getCurrentLevel() == sp.slStop) {
        stop();
     }
      wait(0.5);
    }
    return 0;
  }
  
 private:
   SafetySystem& ss;
   MySafetyProperties& sp;
   DS402 ds402;
   InitSequence init;
   MoveSequence move;
   StopSequence stop;
   Wait wait;
};
