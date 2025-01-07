#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/Fault.hpp>

#include<exception>
#include <array>

#include <execinfo.h>

namespace eeros {
	namespace safety {
		
		uint8_t SafetySystem::instCount = 0;
		SafetySystem* SafetySystem::instance = nullptr;
		
		SafetySystem::SafetySystem(SafetyProperties& safetyProperties, double period) :
		log(logger::Logger::getLogger('S')),
		currentLevel(nullptr),
		privateContext(this),
		period(period) {
			if(++instCount > 1) { // only one instance is allowed
				throw Fault("only one instance of the safety system is allowed");
			}
			if(!setProperties(safetyProperties)) {
				throw Fault("verification of safety properties failed!");
			}
			instance = this;
			std::set_terminate([](){
				try {
					try {
						std::exception_ptr e = std::current_exception();
						if(e) std::rethrow_exception(e);

					} catch (std::exception& e) {
						if(instance)
						{
							instance->log.error() << "uncaught exception: " << e.what();
							std::array<void*, 10> pointers;
							auto num_elems = backtrace(pointers.data(), pointers.size());
							auto symbols = backtrace_symbols(pointers.data(), num_elems);
							for (auto i = 0; i < num_elems; ++i) {
								instance->log.error() << symbols[i];
							}
						}
					}
					if(instance)instance->log.error() << "Terminating";
					if(instance && &(instance->properties.abortFunction)) instance->properties.abortFunction();
					std::exit(EXIT_FAILURE);
				} catch(...) {
					std::abort();
				}
			});
		}
		
		SafetySystem::~SafetySystem() {
			instCount--;
		}

		SafetyLevel& SafetySystem::getCurrentLevel(void) {
			auto level = currentLevel.load(std::memory_order_relaxed);
			if(level) {
				return *level;
			}
			else {
				throw Fault("currentLevel not defined"); // TODO define error number and send error message to logger
			}
		}
		
		bool SafetySystem::setProperties(SafetyProperties& safetyProperties) {
			if(safetyProperties.verify()) {
				properties = safetyProperties;
				currentLevel.store(properties.getEntryLevel(), std::memory_order_relaxed);
				currentLevel.load(std::memory_order_relaxed)->nofActivations = 0;
				nextLevel.store(currentLevel.load(std::memory_order_relaxed), std::memory_order_relaxed);
				log.warn() << "safety system verified: " << (int)properties.levels.size() << " safety levels are present";
				return true;
			}
			return false;
		}
		
		void SafetySystem::triggerEvent(SafetyEvent event, SafetyContext* context) {
			auto current = currentLevel.load(std::memory_order_relaxed);
			auto next = nextLevel.load(std::memory_order_relaxed);
			if(current) {
				SafetyLevel* newLevel = current->getDestLevelForEvent(event, context == &privateContext);
				if(newLevel != nullptr) {
					// prioritize multiple events, 
					// can be called by different threads
					mtx.lock();
					if(next == current) {
						newLevel->nofActivations = 0;
						nextLevel.store(newLevel, std::memory_order_release);
					} else if(newLevel->id < next->id) {
						newLevel->nofActivations = 0;
						nextLevel.store(newLevel, std::memory_order_release);
					}
					mtx.unlock();
					log.info() << "triggering event \'" << event << "\' in level '" << current << "\': transition to safety level: '" << newLevel << "\'";
				} else {
					log.error() << "triggering event \'" << event << "\' in level '" << current << "\': no transition for this event";
				}
			} else {
				throw Fault("current level not defined"); // TODO define error number and send error message to logger
			}
		}
		
		const SafetyProperties* SafetySystem::getProperties() const {
			return &properties;
		}

		double SafetySystem::getPeriod() const {
			return period;
		}

		void SafetySystem::run() {
			// 1) Get currentLevel and nextLevel
			SafetyLevel* level = currentLevel.load(std::memory_order_acquire);
			SafetyLevel* nLevel = nextLevel.load(std::memory_order_acquire);
			if(level != nullptr) {

				level->nofActivations++;
				
				// 2) Read inputs
				for(auto ia : level->inputAction) {
					if(ia != nullptr) {
						if(ia->check(&privateContext)) {
							using namespace logger;
							hal::InputInterface* input = (hal::InputInterface*)(ia->getInput());
							if(level != nLevel) {
								log.info()	<< "level changed due to input action: " << input->getId()
											<< " from level '" << level << "'"
											<< " to level '" << nLevel << "'";
							}
						}
					}
				}
				
				// 3) Execute level action
				if(level->action != nullptr) {
					level->action(&privateContext);
				}
					
				// 4) Set outputs
				for(auto oa : level->outputAction) {
					if(oa != nullptr) {
						oa->set();
					}
				}
				if(nLevel != nullptr && nLevel != level) {
					if(level->onExit) {
						log.info() << "running " << level << "->onExit()";
						level->onExit();
					}
					currentLevel.store(nLevel, std::memory_order_acq_rel);
					if(nLevel->onEntry) {
						log.info() << "running " << nLevel << "->onEntry()";
						nLevel->onEntry(&privateContext);
					}
				}
			}
			else {
				log.error() << "current level is null!";
			}
		}
		
		void SafetySystem::exitHandler() {
			SafetySystem* ss = SafetySystem::instance;
			if(ss) ss->properties.exitFunction(&ss->privateContext);
		}
	};
};
