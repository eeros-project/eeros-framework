#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/task/Lambda.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::hal;
using namespace eeros::task;

double period = 0.01;

class ControlSystem {
 public:
  ControlSystem(double period) : g(10), sw(0), p("out", false), td("td", period, true) {
    // Name block
    c1.setName("constant 1");
    c2.setName("constant 2");
    g.setName("gain");
    i.setName("integrator");
    sw.setName("switch");
    p.setName("analog out");
    
    //name signals
    c1.getOut().getSignal().setName("constant 1 output");
    c2.getOut().getSignal().setName("constant 2 output");
    g.getOut().getSignal().setName("gain output");
    i.getOut().getSignal().setName("integrator output");
    sw.getOut().getSignal().setName("switch output");
    
    // Make connections
    g.getIn().connect(c1.getOut());
    i.getIn().connect(g.getOut());
    sw.getIn(0).connect(i.getOut());
    sw.getIn(1).connect(c2.getOut());
    p.getIn().connect(sw.getOut());
    
    // initialize blocks
    c1.setValue(0.25);
    c2.setValue(-1.5);
    i.setInitCondition(0);
    i.enable();
    sw.setCondition(20, 0.1, 1);
    sw.arm();
    
    // Add blocks to time domain
    td.addBlock(c1);
    td.addBlock(c2);
    td.addBlock(g);
    td.addBlock(i);
    td.addBlock(sw);
    td.addBlock(p);
    
    // add the time domain to the executor
    Executor::instance().add(td);
  }
    
  Constant<> c1, c2;
  Gain<> g;
  I<> i;
  Switch<> sw;
  PeripheralOutput<> p;
  TimeDomain td;
};

class TestSafetyProperties : public SafetyProperties {
 public:
  TestSafetyProperties(ControlSystem& cs) 
      : cs(cs),
        seStartRunning("start running"),
        seStopping("stopping"),
        seRestart("restart running"),
        seDoEmergency("go to emergency"),
        slEmergency("emergency"),
        slInitializing("initializing"),
        slRunning("running"),
        slStopped("stopped") {  
    
    // Add levels
    addLevel(slEmergency);
    addLevel(slInitializing);
    addLevel(slRunning);
    addLevel(slStopped);
      
    // Add events to the levels
    slInitializing.addEvent(seStartRunning, slRunning, kPrivateEvent);
    slRunning.addEvent(seStopping, slStopped, kPublicEvent);
    slStopped.addEvent(seRestart, slRunning, kPrivateEvent);
    addEventToLevelAndAbove(slInitializing, seDoEmergency, slEmergency, kPublicEvent);

    // Define and add level functions
    slEmergency.setLevelAction([&](SafetyContext* privateContext) {
      if(slEmergency.getNofActivations() == 1) cs.td.stop();
    });
      
    slInitializing.setLevelAction([&](SafetyContext* privateContext) {
      cs.td.stop();
      if(slInitializing.getNofActivations() * period > 5) {
        privateContext->triggerEvent(seStartRunning);
        cs.td.start();
      }
    });
    
    slStopped.setLevelAction([&](SafetyContext* privateContext) {
      if(slStopped.getNofActivations() * period > 3) {
        cs.sw.switchToInput(0);
        cs.i.setInitCondition(0);
        cs.sw.arm();
        privateContext->triggerEvent(seRestart);
      }
    });
      
    // Define entry level
    setEntryLevel(slInitializing);  
  }
    
  ControlSystem& cs;
  SafetyEvent seStartRunning, seStopping, seRestart, seDoEmergency;
  SafetyLevel slEmergency, slInitializing, slRunning, slStopped;
};

int main(int argc, char **argv) {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
   
  log.info() << "System test 1 started...";
   
  // Get HAL instance and initialize
  HAL& hal = HAL::instance();
  hal.readConfigFromFile(&argc, argv);
        
  ControlSystem cs(period);
  TestSafetyProperties sp(cs);
  SafetySystem ss(sp, period); 
  // register safety event for control system
  cs.td.registerSafetyEvent(ss, sp.seDoEmergency);
  // register safety event for switch block
  cs.sw.registerSafetyEvent(ss, sp.seStopping);

  Lambda l1 ([&] () { });
  Periodic periodic("per1", 1, l1);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
    log.info() << cs.p.getValue();
  });
    
  // Create and run executor
  auto& executor = eeros::Executor::instance();
  executor.setMainTask(ss);
  executor.add(periodic);
  executor.run();

  log.info() << "Test finished...";
}
