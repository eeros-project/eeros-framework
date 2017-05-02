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
				return false;
			}
			
		private:
			XBox* x;
		};

	};
};

#endif /* ORG_EEROS_XBOX_DIGIN_HPP_ */
