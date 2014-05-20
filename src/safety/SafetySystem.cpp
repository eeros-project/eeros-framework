#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/EEROSException.hpp>

namespace eeros {
	namespace safety {
		
		uint8_t SafetySystem::instCount = 0;
		
		SafetySystem::SafetySystem(SafetyProperties properties, double period) : properties(properties), log('X'), currentLevel(nullptr), privateContext(this),PeriodicThread(period, 0, PeriodicThread::isRealtimeSupported()) {
			currentLevel = properties.entryLevelPtr();
			if(++instCount > 1) { // only one instance is allowed
				throw EEROSException("only one instance of the safety system is allowed");
			}
		}
		
		SafetySystem::~SafetySystem() {
			stop();
			join();
			instCount--;
		}
		
		void SafetySystem::stop() {
			stop();
		}
		
		SafetyLevel& SafetySystem::getCurrentLevel(void) {
			if(currentLevel) {
				return *currentLevel;
			}
			else {
				throw EEROSException("currentLevel not defiend"); // TODO define error number and send error message to logger
			}
		}

		SafetyLevel& SafetySystem::getLevelById(int32_t levelId) {
			for(auto& level : properties.levels) {
				if(level.getId() == levelId) return level;
			}
			throw EEROSException("level is not defined"); // TODO define error number and send error message to logger 
		}

		SafetyLevel& SafetySystem::operator[](unsigned levelId) {
			return getLevelById(levelId);
		}

		void SafetySystem::triggerEvent(uint32_t event, SafetyContext* context) {
			if(currentLevel) {
				log.info() << "triggering event: (" << (int)event << ")";
				int32_t nextLevelId = currentLevel->getLevelIdForEvent(event, context == &privateContext);
				if(nextLevelId != kInvalidLevel) {
					SafetyLevel* nextLevel = &(getLevelById(nextLevelId));
					if(nextLevel != nullptr) {
						bool transition = (nextLevel != currentLevel);
						currentLevel = nextLevel; // TODO make atomic

						if (transition)
							log.info() << "new safety level: [" << nextLevelId << "] " << nextLevel->getDescription();
					}
					else {
						throw EEROSException("unknown safety level"); // TODO define error number and send error message to logger
					}
					
				}
				else {
					log.error()	<< "no transition for event (" << (int)event
								<< ") in level [" << currentLevel->getId() << "] "
								<< currentLevel->getDescription();
				}
			}
			else {
				throw EEROSException("current level not defined"); // TODO define error number and send error message to logger
			}
		}
		
		const SafetyProperties* SafetySystem::getProperties() const {
			return &properties;
		}

		void SafetySystem::run() {
			if(currentLevel != nullptr) {
				// 1) Make local copy of currentLevel
				SafetyLevel* level = currentLevel;
				
				// 2) Read inputs
				for(auto ia : level->inputAction) {
					if(ia != nullptr) {
						SafetyLevel* oldLevel = currentLevel;
						if (ia->check(&privateContext)) {
							SafetyLevel* newLevel = currentLevel;
							using namespace eeros::logger;
							eeros::hal::PeripheralInputInterface* action = (eeros::hal::PeripheralInputInterface*)(ia->inputInterface); // TODO make getId() const
							if (oldLevel != newLevel) {
								log.info()	<< "level changed due to input action: " << action->getId() << endl
											<< "  previous level: [" << oldLevel->getId() << "] " << oldLevel->getDescription() << endl
											<< "  new level:      [" << newLevel->getId() << "] " << newLevel->getDescription();
							} else {
	//							log.trace()	<< "no change by input action: " << action->getId();
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
			}
			else {
				log.error() << "current level is null!";
			}
		}
	};
};
