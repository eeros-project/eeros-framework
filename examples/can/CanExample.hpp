#include <iostream>
#include <signal.h>

#include <eeros/control/Gain.hpp>

#include <eeros/control/can/CanHandle.hpp>
#include <eeros/control/can/CanReceiveFaulhaber.hpp>
#include <eeros/control/can/CanSendFaulhaber.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::safety;

static constexpr int node1 = 1;           // id of node 1
static constexpr int node2 = 2;           // id of node 2

class MyControlSystem {
 public:
  MyControlSystem(double ts) 
      : handle("can1"), 
        canReceive(handle.getSocket(), {node1, node2}, {CANOPEN_FC_PDO1_TX, CANOPEN_FC_PDO2_TX}),
        canSend(handle.getSocket(), {node1, node2}),
        timedomain("Main time domain", ts, true) {
    timedomain.addBlock(canReceive);        
    timedomain.addBlock(canSend);        
    Executor::instance().add(timedomain);
  }
    
  CanHandle handle;
  CanReceiveFaulhaber canReceive;
  CanSendFaulhaber canSend;
  TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
 public:
  MySafetyProperties(MyControlSystem& cs) : slOff("off"), cs(cs) {    
    addLevel(slOff);
    setEntryLevel(slOff);
    slOff.setLevelAction([&](SafetyContext* privateContext) {
      if ((slOff.getNofActivations() % 5) == 0) {
        log.warn() << cs.canReceive.getOutStatus(node1).getSignal();
        log.warn() << cs.canReceive.getOutPos(node2).getSignal();
      }
    });
  }
    
  SafetyLevel slOff;
  MyControlSystem& cs;
  Logger log;
};