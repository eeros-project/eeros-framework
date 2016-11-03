#ifndef ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_
#define ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_

#include <stdint.h>

namespace eeros {
	namespace safety {

		class SafetySystem;
		class SafetyEvent;

		class SafetyContext {
			friend class SafetySystem;
			
		public:
			void triggerEvent(SafetyEvent event);
			
		private:
			SafetyContext(SafetySystem* parent);
			SafetySystem* parent;
		};

	};
};

#endif // ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_
