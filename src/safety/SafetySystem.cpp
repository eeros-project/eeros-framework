#include <cxxabi.h>
#include <execinfo.h>

#include <array>
#include <eeros/core/Fault.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/InputAction.hpp>
#include <exception>
#include <typeinfo>

namespace eeros {
namespace safety {

uint8_t SafetySystem::instCount = 0;
SafetySystem* SafetySystem::instance = nullptr;

SafetySystem::SafetySystem(SafetyProperties& safetyProperties, double period)
    : log(logger::Logger::getLogger('S')),
      currentLevel(nullptr),
      privateContext(this),
      period(period) {
  if (++instCount > 1) {  // only one instance is allowed
    throw Fault("only one instance of the safety system is allowed");
  }
  if (!setProperties(safetyProperties)) {
    throw Fault("verification of safety properties failed!");
  }
  instance = this;
  std::set_terminate([]() {
    try {
      try {
        std::exception_ptr e = std::current_exception();
        if (e) std::rethrow_exception(e);

      } catch (std::exception& e) {
        if (instance) {
          int status;
          auto et = abi::__cxa_demangle(typeid(e).name(), NULL, NULL, &status);
          instance->log.error()
              << "uncaught exception: " << (status == 0 ? et : "") << ' '
              << e.what();
        }
      }
      if (instance) {
        printStackTrace();
        instance->log.error() << "Terminating";
        if (instance->properties.abortFunction)
          instance->properties.abortFunction();
      }

      std::exit(EXIT_FAILURE);
    } catch (...) {
      if (instance) instance->log.error() << "Aborting";
      std::abort();
    }
  });
}

SafetySystem::~SafetySystem() { instCount--; }

SafetyLevel& SafetySystem::getCurrentLevel(void) {
  auto cLevel = currentLevel.load(std::memory_order_relaxed);
  auto eLevel = entryLevel.load(std::memory_order_relaxed);
  if (cLevel)
    return *cLevel;
  else if (eLevel)
    return *eLevel;
  else
    throw Fault("currentLevel not defined");  // TODO define error number and
                                              // send error message to logger
}

bool SafetySystem::setProperties(SafetyProperties& safetyProperties) {
  if (safetyProperties.verify()) {
    properties = safetyProperties;
    auto eLevel = properties.getEntryLevel();
    eLevel->nofActivations = 0;
    entryLevel.store(eLevel, std::memory_order_release);
    nextLevel.store(eLevel, std::memory_order_release);
    log.warn() << "safety system verified: " << (int)properties.levels.size()
               << " safety levels are present";
    return true;
  }
  return false;
}

void SafetySystem::triggerEvent(SafetyEvent event, SafetyContext* context) {
  auto current = currentLevel.load(std::memory_order_acquire);
  if (current) {
    SafetyLevel* newLevel =
        current->getDestLevelForEvent(event, context == &privateContext);
    if (newLevel != nullptr) {
      SafetyLevel* expected = nullptr;
      // using std::memory_order_relaxed should be ok even though we access
      // ->id, since the ids should be constant
      while (!nextLevel.compare_exchange_strong(expected, newLevel,
                                                std::memory_order_relaxed)) {
        // prioritize "lower" levels (i.e. levels with a lower id)
        // so only keep trying to store our level, if the existing one isn't
        // lower
        if (expected->id < newLevel->id) return;
      }

      log.info() << "triggering event \'" << event << "\' in level '" << current
                 << "\': transition to safety level: '" << newLevel << "\'";
    } else {
      log.error() << "triggering event \'" << event << "\' in level '"
                  << current << "\': no transition for this event";
    }
  } else {
    throw Fault("current level not defined");  // TODO define error number and
                                               // send error message to logger
  }
}

const SafetyProperties* SafetySystem::getProperties() const {
  return &properties;
}

double SafetySystem::getPeriod() const { return period; }

void SafetySystem::run() {
  // 1) Get currentLevel
  SafetyLevel* level = currentLevel.load(std::memory_order_acquire);

  // 2) Set new level
  SafetyLevel* nLevel = nullptr;
  while (!nextLevel.compare_exchange_strong(nLevel, nullptr,
                                            std::memory_order_acquire))
    ;

  if (nLevel != nullptr && nLevel != level) {
    if (level && level->onExit) {
      log.info() << "running " << level << "->onExit()";
      level->onExit();
    }
    currentLevel.store(nLevel, std::memory_order_acq_rel);
    level = nLevel;
    if (nLevel->onEntry) {
      log.info() << "running " << nLevel << "->onEntry()";
      nLevel->onEntry(&privateContext);
    }
  }

  if (level != nullptr) {
    level->nofActivations++;

    // 3) Read inputs
    for (auto ia : level->inputAction) {
      if (ia != nullptr) {
        if (ia->check(&privateContext)) {
          using namespace logger;
          hal::InputInterface* input = (hal::InputInterface*)(ia->getInput());
          log.info() << "input action triggered: " << input->getId();
        }
      }
    }

    // 4) Execute level action
    if (level->action != nullptr) {
      level->action(&privateContext);
    }

    // 5) Set outputs
    for (auto oa : level->outputAction) {
      if (oa != nullptr) {
        oa->set();
      }
    }

  } else {
    log.error() << "current level is null!";
  }
}

void SafetySystem::exitHandler() {
  SafetySystem* ss = SafetySystem::instance;
  if (ss) ss->properties.exitFunction(&ss->privateContext);
}

void SafetySystem::printStackTrace() {
  std::array<void*, 20> pointers = {};
  auto num_elems = backtrace(pointers.data(), pointers.size());
  auto symbols = backtrace_symbols(pointers.data(), num_elems);
  for (int i = 0; i < num_elems; ++i) {
    // find mangled function name substring
    int start = 0;
    while (symbols[i][start] != '(' && symbols[i][start] != 0) ++start;
    ++start;
    int end = start;
    for (char c = symbols[i][start]; c != '+' && c != 0; c = symbols[i][end])
      ++end;

    // demangle if we have a symbol name
    if (start == end || end == start + 1) {
      instance->log.error() << i << ": " << symbols[i];
    } else {
      // C string shenanigans to splice the demangled name into the output
      // without copying strings
      symbols[i][end] = '\0';
      if (start != 0) symbols[i][start - 1] = '\0';
      // demangle name
      int status;
      auto demangled =
          abi::__cxa_demangle(&symbols[i][start], NULL, NULL, &status);

      // print demangled name if we can
      if (status == 0)
        instance->log.error() << i << ": " << symbols[i] << '(' << demangled
                              << '+' << &symbols[i][end + 1];
      else
        instance->log.error()
            << i << ": " << symbols[i] << '(' << &symbols[i][start] << '+'
            << &symbols[i][end + 1];
    }
  }
}
};  // namespace safety
};  // namespace eeros

