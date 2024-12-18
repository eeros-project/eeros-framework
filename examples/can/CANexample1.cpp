/**
 * Demo program using two myactuator drives communicating over
 * socket CAN using basic CAN frames.
 */

#include <eeros/control/can/CANsend.hpp>
#include <eeros/control/can/CANreceive.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <signal.h>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::sequencer;

static constexpr uint8_t node1 = 1;           // id of node 1
static constexpr uint8_t node2 = 2;           // id of node 2
static constexpr uint8_t nofNodes = 2;        // nof nodes

class ControlSystem {
public:
  ControlSystem(double ts)
      : canHandle(std::make_shared<CAN::CANSocket>("can0")),
        canSend(canHandle, {node1, node2}),
        canReceive(canHandle, {node1, node2}),
        timedomain("Main time domain", ts, true) {
    pos.setValue({0, 0});
    canSend.setName("CAN send");
    canReceive.setName("CAN receive");
    canReceive.getOut().getSignal().setName("position");
    Matrix<2,1,double> scale({10, 10});    // accounts for encoder resolution and gear box
    canSend.setScale(scale);
    canSend.getIn().connect(pos.getOut());

    timedomain.addBlock(canReceive);
    timedomain.addBlock(pos);
    timedomain.addBlock(canSend);

    Executor::instance().add(timedomain);
  }

  CAN canHandle;
  CANsend<nofNodes> canSend;
  CANreceive<nofNodes> canReceive;
  Constant<Matrix<nofNodes,1,double>> pos;
  TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
public:
  MySafetyProperties()
  : slInit("init"),
  slMoving("moving"),
  slShutDown("shut down"),
  seStartMoving("start moving"),
  seShutDown("shutting down") {
    addLevel(slShutDown);
    addLevel(slInit);
    addLevel(slMoving);
    slInit.addEvent(seStartMoving, slMoving, kPublicEvent);
    slMoving.addEvent(seShutDown, slShutDown, kPublicEvent);

    slInit.setLevelAction([&](SafetyContext* privateContext) {privateContext->triggerEvent(seStartMoving);});
    slShutDown.setLevelAction([&](SafetyContext* privateContext) {Executor::stop();});

    setEntryLevel(slInit);

    exitFunction = [&](SafetyContext* privateContext) {
      privateContext->triggerEvent(seShutDown);
    };
  }

  SafetyLevel slInit, slMoving, slShutDown;
  SafetyEvent seStartMoving, seShutDown;
};

class MainSequence : public Sequence {
public:
  MainSequence(std::string name, Sequencer& seq, ControlSystem& cs, SafetySystem& ss, MySafetyProperties& sp)
  : Sequence(name, seq), cs(cs), ss(ss), sp(sp), wait("Wait", this) { }

  int action() {
    Vector2 speed{5000, 5000};
    cs.canSend.setSpeed(speed);
    cs.canSend.enable();
    cs.canReceive.enable();
    while(Sequencer::running) {
      if (ss.getCurrentLevel() == sp.slMoving) {
        cs.pos.setValue({500, 300});
        wait(5);
        cs.pos.setValue({1000, 400});
        wait(5);
        cs.pos.setValue({0, 0});
        wait(5);
      }
    }
    return 0;
  }

private:
  ControlSystem& cs;
  SafetySystem& ss;
  MySafetyProperties& sp;
  Wait wait;
};

void signalHandler(int signum) {
  Sequencer::instance().abort();
  SafetySystem::exitHandler();
}

int main(int argc, char **argv) {
  double dt = 0.005;
  signal(SIGINT, signalHandler);

  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger('M');

  log.info() << "CAN demo";

  ControlSystem cs(dt);
  MySafetyProperties sp;
  SafetySystem ss(sp, dt);

  auto& sequencer = Sequencer::instance();
  MainSequence mainSeq("Main Sequence", sequencer, cs, ss, sp);
  mainSeq();

  auto& executor = Executor::instance();
  executor.setMainTask(ss);

  task::Lambda l1 ([&] () { });
  task::Periodic perLog("periodic log", 0.5, l1);
  perLog.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
    log.info() << cs.canReceive.getOut().getSignal();
  });
  executor.add(perLog);
  executor.run();

  sequencer.wait();

  log.info() << "Motor test end";
  return 0;
}

