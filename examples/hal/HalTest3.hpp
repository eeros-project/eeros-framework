#ifndef ORG_EEROS_HALTEST3_HPP_
#define ORG_EEROS_HALTEST3_HPP_

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/control/Watchdog.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/safety/InputAction.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::task;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::hal;

class ControlSystem {
 public:
  ControlSystem(double ts) : in("button1"), out("led1"), td("Main time domain", ts, true) {
    out.getIn().connect(in.getOut());
    td.addBlock(wdt);
    td.addBlock(in);
    td.addBlock(out);
    Executor::instance().add(td);
  }

  Watchdog wdt;
  PeripheralOutput<bool> out;		// digital output
  PeripheralInput<bool> in;		// digital input
  TimeDomain td;
};

class TestSafetyProperties : public SafetyProperties {
 public:
  TestSafetyProperties(ControlSystem& cs, double ts)
      : slOff("off"),
        slReady("ready"),
        slMoving("moving"),
        abort("abort"),
        startMoving("start moving"),
        cs(cs) {
    wdt = HAL::instance().getLogicInput("Wdt", false);
    criticalInputs = {wdt};
    slOff.setInputActions({ignore(wdt)});
    slReady.setInputActions({ignore(wdt)});
    slMoving.setInputActions({check(wdt, true, abort)});

    addLevel(slOff);
    addLevel(slReady);
    addLevel(slMoving);

    slReady.addEvent(startMoving, slMoving, kPublicEvent);
    addEventToLevelAndAbove(slReady, abort, slOff, kPrivateEvent);

    slOff.setLevelAction([&](SafetyContext* privateContext) {
		eeros::Executor::stop();
	});
    slReady.setLevelAction([&](SafetyContext* privateContext) {
		cs.wdt.arm();
		privateContext->triggerEvent(startMoving);
	});

    setEntryLevel(slReady);

    exitFunction = ([&](SafetyContext* privateContext){
      privateContext->triggerEvent(abort);
    });
  }

  SafetyLevel slOff, slReady, slMoving;
  SafetyEvent abort, startMoving;
  ControlSystem& cs;
  hal::Input<bool>* wdt;
};

#endif // ORG_EEROS_HALTEST3_HPP_
