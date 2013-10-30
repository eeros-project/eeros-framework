#ifndef ORG_EEROS_SAFETY_INPUTACTION_HPP_
#define ORG_EEROS_SAFETY_INPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/SystemInput.hpp>

namespace eeros {
	namespace safety {

		class SafetyContext;

		class InputAction {
		public:
			InputAction(eeros::hal::SystemInputInterface& inputInterface) : inputInterface(&inputInterface) { }
			virtual ~InputAction() { }
			virtual void check(SafetyContext* context) { }
			
			eeros::hal::SystemInputInterface const* inputInterface;
		};

	};
};

#endif // ORG_EEROS_SAFETY_INPUTACTION_HPP_