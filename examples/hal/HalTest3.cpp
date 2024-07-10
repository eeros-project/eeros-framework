#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "HalTest3.hpp"

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::task;
using namespace eeros::hal;
using namespace eeros::safety;

int main(int argc, char **argv) {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();

  log.info() << "test started...";

  HAL& hal = HAL::instance();
  hal.readConfigFromFile(&argc, argv);
  double period = 0.5;
  ControlSystem cs(period);
  TestSafetyProperties sp(cs, period);
  SafetySystem ss(sp, period);

  auto& executor = Executor::instance();
  executor.setMainTask(ss);
  Lambda l1 ([&] () {log.warn() << "status=" << cs.wdt.getStatus();});
  Periodic periodic("per1", 1, l1);
  executor.add(periodic);

  executor.run();

  log.info() << "test finished...";
  return 0;
}

