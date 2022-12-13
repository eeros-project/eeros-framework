#include <eeros/control/TimeDomain.hpp>
#include <iostream>

using namespace eeros::control;

TimeDomain::TimeDomain(std::string name, double period, bool realtime)
    : handle(nullptr), name(name), period(period), realtime(realtime), safetySystem(nullptr), safetyEvent(nullptr) { }

TimeDomain::TimeDomain(const rclcpp::Node::SharedPtr node, std::string name, double period, bool realtime)
    : handle(node), name(name), period(period), realtime(realtime), safetySystem(nullptr), safetyEvent(nullptr) { }

std::string TimeDomain::getName() {
  return name;
}

double TimeDomain::getPeriod() {
  return period;
}

bool TimeDomain::getRealtime() {
  return realtime;
}

void TimeDomain::registerSafetyEvent(SafetySystem& ss, SafetyEvent& e) {
  safetySystem = &ss;
  safetyEvent = &e;
}

void TimeDomain::run() {
  if (!running) {
    return;
  }
  try {
    for (auto block : blocks) {
      block->run();
    }

  } catch (NotConnectedFault const& e) {
    if (safetySystem != nullptr && safetyEvent != nullptr) {
      safetySystem->triggerEvent(*safetyEvent);
      safetySystem->log.error() << e.what();
    } else {
      throw eeros::Fault(std::string(e.what()) + ", time domain cannot trigger safety event");
    }
  } catch (NaNOutputFault const& e) {
    if (safetySystem != nullptr && safetyEvent != nullptr) {
      safetySystem->triggerEvent(*safetyEvent);
      safetySystem->log.error() << e.what();
    } else {
      throw eeros::Fault(std::string(e.what()) + ", time domain cannot trigger safety event");
    }
  }
}

void TimeDomain::start() {
  running = true;
}

void TimeDomain::stop() {
  running = false;
}

void TimeDomain::addBlock(eeros::Runnable* block) {
  blocks.push_back(block);
}

void TimeDomain::addBlock(eeros::Runnable& block) {
  blocks.push_back(&block);
}

void TimeDomain::removeBlock(eeros::Runnable* block) {
  blocks.remove(block);
}

void TimeDomain::removeBlock(eeros::Runnable& block) {
  blocks.remove(&block);
}

namespace eeros {
namespace control {
  
std::ostream& operator<<(std::ostream& os, TimeDomain& td) {
  os << "Time domain: '" << td.getName() << "'"; 
  return os;
}
  
}
}
