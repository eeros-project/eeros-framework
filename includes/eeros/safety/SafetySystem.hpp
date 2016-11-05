#ifndef ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
#define ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_

#include <vector>
#include <eeros/core/Runnable.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetyContext.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

namespace eeros {
	namespace safety {
		
		// Forward declarations
		class SystemInputInterface;
		class SystemOutputInterface;
		
		class SafetySystem : public Runnable {
		public:
			SafetySystem(SafetyProperties& properties, double period);
			virtual ~SafetySystem();
			SafetyLevel& getCurrentLevel(void);
			void triggerEvent(SafetyEvent event, SafetyContext* context = nullptr);
			const SafetyProperties* getProperties() const;
			double getPeriod() const;
			void run();
			static void exitHandler();
			logger::Logger<logger::LogWriter> log;
			
		private:
			bool setProperties(SafetyProperties& safetyProperties);
			
			SafetyProperties properties;
			SafetyLevel* currentLevel;
			SafetyLevel* nextLevel;
			SafetyContext privateContext;
			static uint8_t instCount;
			static SafetySystem* instance;
			double period;
		};

	};
};

#endif // ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
