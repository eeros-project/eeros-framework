#ifndef ORG_EEROS_SAFETY_INPUTACTION_HPP_
#define ORG_EEROS_SAFETY_INPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/PeripheralInput.hpp>

namespace eeros {
	namespace safety {

		class SafetyContext;

		class InputAction {
		public:
			InputAction(eeros::hal::PeripheralInputInterface& inputInterface) : inputInterface(&inputInterface) { }
			virtual ~InputAction() { }
			virtual bool check(SafetyContext* context) { return false; }
			
			eeros::hal::PeripheralInputInterface const* inputInterface;
		};

	};
};

#endif // ORG_EEROS_SAFETY_INPUTACTION_HPP_
