#ifndef ORG_EEROS_SAFETY_INPUTACTION_HPP_
#define ORG_EEROS_SAFETY_INPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/Input.hpp>

namespace eeros {
	namespace safety {

		class SafetyContext;

		class InputAction {
		public:
			InputAction(eeros::hal::InputInterface& inputInterface) : inputInterface(&inputInterface) { }
			virtual ~InputAction() { }
			virtual bool check(SafetyContext* context) { return false; }
			
			eeros::hal::InputInterface const* inputInterface;
		};

	};
};

#endif // ORG_EEROS_SAFETY_INPUTACTION_HPP_
