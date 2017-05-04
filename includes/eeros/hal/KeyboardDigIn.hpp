#ifndef ORG_EEROS_KEYBOARD_DIGIN_HPP_
#define ORG_EEROS_KEYBOARD_DIGIN_HPP_

#include <eeros/hal/Input.hpp>
#include <eeros/hal/Mouse.hpp>
#include <fstream>
#include <string>

namespace eeros {
	namespace hal {
		class KeyboardDigIn : public Input<bool> {
		public:
			KeyboardDigIn(std::string id, Keyboard* k) : Input<bool>(id, nullptr), k(k) { }
			~KeyboardDigIn() { }
			virtual bool get() {
				if (getId().compare("escKeyboardButton") == 0) return k->events.esc;
				if (getId().compare("emergencyKeyboardButton") == 0) return k->events.emergency;
				if (getId().compare("resetKeyboardButton") == 0) return k->events.reset;
				if (getId().compare("startKeyboardButton") == 0) return k->events.start;
				if (getId().compare("stopKeyboardButton") == 0) return k->events.stop;
				return false;
			}
			
		private:
			Keyboard* k;
		};

	};
};

#endif /* ORG_EEROS_KEYBOARD_DIGIN_HPP_ */
