#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/EEROSException.hpp>

namespace eeros {
	namespace safety {
		
		uint8_t SafetySystem::instCount = 0;
		SafetySystem* SafetySystem::instance = nullptr;
		
		SafetySystem::SafetySystem(SafetyProperties& safetyProperties, double period) :
		log('S'),
		currentLevel(nullptr),
		privateContext(this),
		period(period) {
			if(++instCount > 1) { // only one instance is allowed
				throw EEROSException("only one instance of the safety system is allowed");
			}
			if(!setProperties(safetyProperties)) {
				throw EEROSException("verification of safety properties failed!");
			}
			instance = this;
		}
		
		SafetySystem::~SafetySystem() {
			instCount--;
		}

		SafetyLevel& SafetySystem::getCurrentLevel(void) {
			if(currentLevel) {
				return *currentLevel;
			}
			else {
				throw EEROSException("currentLevel not defined"); // TODO define error number and send error message to logger
			}
		}
		
		bool SafetySystem::setProperties(SafetyProperties& safetyProperties) {
			if(safetyProperties.verify()) {
				properties = safetyProperties;
				currentLevel = properties.getEntryLevel();
				nextLevel = currentLevel;
				return true;
			}
			return false;
		}
		
		void SafetySystem::triggerEvent(SafetyEvent event, SafetyContext* context) {
			if (currentLevel) {
				log.info() << "triggering event: \'" << event.getDescription() << "\' in level '" << nextLevel->getDescription() << "\'";
				nextLevel = currentLevel->getDestLevelForEvent(event, context == &privateContext);
				if (nextLevel != nullptr) {
					bool transition = (nextLevel != currentLevel);	// stage level change
					if (transition) log.info() << "new safety level: '" << nextLevel->getDescription() << "\'";	
				} else {
					log.error()	<< "no transition for event \'" << event.getDescription()
								<< "\' in level \'" << currentLevel->getDescription() << "\'";
				}
			} else {
				throw EEROSException("current level not defined"); // TODO define error number and send error message to logger
			}
		}
		
		const SafetyProperties* SafetySystem::getProperties() const {
			return &properties;
		}

		double SafetySystem::getPeriod() const {
			return period;
		}

		void SafetySystem::run() {
			// level must only change before safety system runs or after run method has finished
			if (nextLevel != nullptr) currentLevel = nextLevel; // TODO make atomic
			if(currentLevel != nullptr) {

				// 1) Get currentLevel
				SafetyLevel* level = currentLevel;
				
				// 2) Read inputs
				for(auto ia : level->inputAction) {
					if(ia != nullptr) {
						SafetyLevel* oldLevel = currentLevel;
						if (ia->check(&privateContext)) {
							SafetyLevel* newLevel = nextLevel;
							using namespace logger;
							hal::PeripheralInputInterface* input = (hal::PeripheralInputInterface*)(ia->getInput());
							if (oldLevel != newLevel) {
								log.info()	<< "level changed due to input action: " << input->getId() << endl
											<< "  previous level: '" << oldLevel->getDescription() << "'" << endl
											<< "  new level:      '" << newLevel->getDescription() << "'";
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
				if (nextLevel != nullptr) currentLevel = nextLevel; // TODO make atomic
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
