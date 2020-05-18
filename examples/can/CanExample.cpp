#include <iostream>
#include <signal.h>

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
        canReceive(handle.getSocket(), {node1, node2}, {CANOPEN_FC_PDO1_TX, CANOPEN_FC_PDO1_RX , CANOPEN_FC_PDO2_RX, CANOPEN_FC_PDO2_TX, CANOPEN_FC_PDO3_TX, CANOPEN_FC_PDO3_RX, CANOPEN_FC_PDO4_TX, CANOPEN_FC_PDO4_RX}),
        canSend(handle.getSocket(), {node1, node2}),
        timedomain("Main time domain", ts, true) {
    timedomain.addBlock(canReceive);        
    timedomain.addBlock(canSend);        
    Executor::instance().add(timedomain);
  }
    
 private:
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
            }
        });
    }
    
    SafetyLevel slOff;
    MyControlSystem cs;
    Logger log;
};

// void signalHandler(int signum) {
// 	Executor::stop();
// }
// 


int main(int argc, char **argv) {
  double dt = 0.2;

  StreamLogWriter w(std::cout);
  Logger::setDefaultWriter(&w);
  Logger log;
  w.show();
 
  log.info() << "CAN test start";

  MyControlSystem cs(dt);
  MySafetyProperties safetyProperties(cs);
// 	eeros::safety::SafetySystem safetySystem(safetyProperties, dt);
	
// 	signal(SIGINT, signalHandler);	
	auto& executor = Executor::instance();
// 	executor.setMainTask(safetySystem);
// // 	executor.syncWithRosTopic(&syncCallbackQueue);	// sync with gazebo simulation
	executor.run();

	log.info() << "CAN test end";	
	return 0;
}
