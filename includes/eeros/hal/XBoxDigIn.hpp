#ifndef ORG_EEROS_XBOX_DIGIN_HPP_
#define ORG_EEROS_XBOX_DIGIN_HPP_

#include <eeros/hal/Input.hpp>
#include <eeros/hal/XBox.hpp>
#include <fstream>
#include <string>

namespace eeros {
	namespace hal {
		class XBoxDigIn : public Input<bool> {
		public:
			XBoxDigIn(std::string id, XBox* x) : Input<bool>(id, nullptr), x(x) { }
			~XBoxDigIn() { }
			virtual bool get() {
				if (getId().compare("XBoxButtonA") == 0) return x->current.button_state[XBoxController::Button::A];
				if (getId().compare("XBoxButtonB") == 0) return x->current.button_state[XBoxController::Button::B];
				if (getId().compare("XBoxButtonX") == 0) return x->current.button_state[XBoxController::Button::X];
				if (getId().compare("XBoxButtonY") == 0) return x->current.button_state[XBoxController::Button::Y];
				if (getId().compare("XBoxButtonLB") == 0) return x->current.button_state[XBoxController::Button::LB];
				if (getId().compare("XBoxButtonRB") == 0) return x->current.button_state[XBoxController::Button::RB];
				if (getId().compare("XBoxButtonBack") == 0) return x->current.button_state[XBoxController::Button::back];
				if (getId().compare("XBoxButtonStart") == 0) return x->current.button_state[XBoxController::Button::start];
				return false;
			}
			
		private:
			XBox* x;
		};

	};
};

#endif /* ORG_EEROS_XBOX_DIGIN_HPP_ */
