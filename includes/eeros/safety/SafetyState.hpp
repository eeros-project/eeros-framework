#ifndef ORG_EEROS_SAFETY_SAFETYSTATE_HPP_
#define ORG_EEROS_SAFETY_SAFETYSTATE_HPP_

#include <vector>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/hal/HAL.hpp>

namespace eeros {
	namespace safety {

		class SafetyState {
		public:
			SafetyState();
			SafetyLevel& getLevel(int32_t levelId);
			
			std::vector<SafetyLevel> levels;
			SafetyLevel* currentLevel;
			
			std::vector<eeros::hal::SystemOutputInterface*> criticalOutputs;
			std::vector<eeros::hal::SystemInputInterface*> criticalInputs;
		};

	};
};

#endif // ORG_EEROS_SAFETY_SAFETYSTATE_HPP_