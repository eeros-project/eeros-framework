#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/hal/HAL.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::hal;
using namespace eeros::control;

class SafetyPropertiesTestc1 : public SafetyProperties {
 public:
  SafetyPropertiesTestc1()  
      : se1("se1"), se2("se2"),
        sl1("1"), sl2("2") {
    addLevel(sl1);
    addLevel(sl2);
    sl1.addEvent(se1, sl2, kPublicEvent);
    sl2.addEvent(se2, sl1, kPublicEvent);

    out = HAL::instance().getLogicOutput("io1", false);
    criticalOutputs = { out };
    in = HAL::instance().getLogicInput("ioIn", false);
    criticalInputs = { in };
    sl1.setInputActions({ check(in, false, se1) });
    sl2.setInputActions({ check(in, true, se2) });

    // Define output states and events for all levels
    sl1.setOutputActions({ set(out, true) });
    sl2.setOutputActions({ set(out, false) });

    setEntryLevel(sl1);
  }
  
  SafetyEvent se1, se2;
  SafetyLevel sl1, sl2;
  hal::Output<bool>* out;
  hal::Input<bool>* in;
};


// test critical in and outputs
TEST(safetyCriticalTest, digitalInput) {
  std::cout.setstate(std::ios_base::badbit);
  logger::Logger::setDefaultStreamLogger(std::cout);
  SafetyPropertiesTestc1 sp;
  SafetySystem ss(sp, 1);
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  Constant<bool> c(false);
  PeripheralInput<bool> pi0("ioIn0", false);
  PeripheralOutput<bool> p("ioOut2", false);
  PeripheralInput<bool> pi("ioIn", false);
  p.getIn().connect(c.getOut());
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_EQ(pi.getOut().getSignal().getValue(), false);
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  EXPECT_EQ(pi0.getOut().getSignal().getValue(), true);
  c.setValue(true);
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_EQ(pi.getOut().getSignal().getValue(), true);
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
  EXPECT_EQ(pi0.getOut().getSignal().getValue(), false);
  ss.run();
  pi0.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
  c.setValue(false);
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_EQ(pi.getOut().getSignal().getValue(), false);
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  EXPECT_EQ(pi0.getOut().getSignal().getValue(), true);
  HAL::instance().releaseOutput("io1"); // release for further hal tests
  HAL::instance().releaseInput("ioIn");
}

class SafetyPropertiesTestc2 : public SafetyProperties {
 public:
  SafetyPropertiesTestc2()  
      : se1("se1"), se2("se2"),
        sl1("1"), sl2("2") {
    addLevel(sl1);
    addLevel(sl2);
    sl1.addEvent(se1, sl2, kPublicEvent);
    sl2.addEvent(se2, sl1, kPublicEvent);

    out = HAL::instance().getScalableOutput("aOut0", false);
    criticalOutputs = { out };
    in = HAL::instance().getScalableInput("aIn1", false);
    criticalInputs = { in };
    sl1.setInputActions({ range(in, 0.9, 1.1, se1) });
    sl2.setInputActions({ range(in, 2.9, 3.1, se2) });

    // Define output states and events for all levels
    sl1.setOutputActions({ set(out, 2.3) });
    sl2.setOutputActions({ set(out, -1.4) });

    setEntryLevel(sl1);
  }
  
  SafetyEvent se1, se2;
  SafetyLevel sl1, sl2;
  hal::Output<double>* out;
  hal::Input<double>* in;
};

// test critical in and outputs
TEST(safetyCriticalTest, analogInput) {
  std::cout.setstate(std::ios_base::badbit);
  logger::Logger::setDefaultStreamLogger(std::cout);
  SafetyPropertiesTestc2 sp;
  SafetySystem ss(sp, 1);
  Constant<> c(1.0);
  PeripheralInput<> pi0("aIn0", false);
  PeripheralOutput<> p("aOut1", false);
  PeripheralInput<> pi("aIn1", false);
  p.getIn().connect(c.getOut());
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(Utils::compareApprox(pi.getOut().getSignal().getValue(), 1.0, 0.01));
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  EXPECT_TRUE(Utils::compareApprox(pi0.getOut().getSignal().getValue(), 2.4, 0.1));
  c.setValue(3.0);
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(Utils::compareApprox(pi.getOut().getSignal().getValue(), 3.0, 0.01));
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
  EXPECT_TRUE(Utils::compareApprox(pi0.getOut().getSignal().getValue(), -1.5, 0.1));
  ss.run();
  pi0.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
  c.setValue(1.0);
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(Utils::compareApprox(pi.getOut().getSignal().getValue(), 1.0, 0.01));
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  EXPECT_TRUE(Utils::compareApprox(pi.getOut().getSignal().getValue(), 1.0, 0.01));
}

class SafetyPropertiesTestc3 : public SafetyProperties {
 public:
  SafetyPropertiesTestc3()  
      : se1("se1"), se2("se2"),
        sl1("1"), sl2("2") {
    addLevel(sl1);
    addLevel(sl2);
    sl1.addEvent(se1, sl2, kPublicEvent);
    sl2.addEvent(se2, sl1, kPublicEvent);

    out = HAL::instance().getScalableOutput("aOut0", false);
    criticalOutputs = { out };
    in = HAL::instance().getScalableInput("aIn1", false);
    criticalInputs = { in };
    sl1.setInputActions({ range(in, 1.5, 3.5, se1, true) });
    sl2.setInputActions({ range(in, 2.9, 3.1, se2) });

    // Define output states and events for all levels
    sl1.setOutputActions({ set(out, 2.3) });
    sl2.setOutputActions({ set(out, -1.4) });

    setEntryLevel(sl1);
  }
  
  SafetyEvent se1, se2;
  SafetyLevel sl1, sl2;
  hal::Output<double>* out;
  hal::Input<double>* in;
};

// test critical in and outputs
TEST(safetyCriticalTest, analogInputOffRange) {
  std::cout.setstate(std::ios_base::badbit);
  logger::Logger::setDefaultStreamLogger(std::cout);
  SafetyPropertiesTestc3 sp;
  SafetySystem ss(sp, 1);
  Constant<> c(1.0);
  PeripheralInput<> pi0("aIn0", false);
  PeripheralOutput<> p("aOut1", false);
  PeripheralInput<> pi("aIn1", false);
  p.getIn().connect(c.getOut());
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
//   EXPECT_EQ(pi.getOut().getSignal().getValue(), 1.0);
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(Utils::compareApprox(pi.getOut().getSignal().getValue(), 1.0, 0.01));
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  EXPECT_TRUE(Utils::compareApprox(pi0.getOut().getSignal().getValue(), 2.4, 0.1));
  c.setValue(3.0);
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(Utils::compareApprox(pi.getOut().getSignal().getValue(), 3.0, 0.01));
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
  EXPECT_TRUE(Utils::compareApprox(pi0.getOut().getSignal().getValue(), -1.5, 0.1));
//   EXPECT_EQ(pi0.getOut().getSignal().getValue(), 10);
  ss.run();
  pi0.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
  c.setValue(1.0);
  c.run(); p.run();
  usleep(10000); // simulator thread needs time to run
  pi.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(Utils::compareApprox(pi.getOut().getSignal().getValue(), 1.0, 0.01));
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  ss.run();
  usleep(10000); // simulator thread needs time to run
  pi0.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
  EXPECT_TRUE(Utils::compareApprox(pi.getOut().getSignal().getValue(), 1.0, 0.01));
  HAL::instance().releaseOutput("aOut0");
  HAL::instance().releaseOutput("aOut1");
  HAL::instance().releaseInput("aIn0");
  HAL::instance().releaseInput("aIn1");
}
