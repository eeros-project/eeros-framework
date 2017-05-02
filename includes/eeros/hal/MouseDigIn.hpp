#ifndef ORG_EEROS_MOUSE_DIGIN_HPP_
#define ORG_EEROS_MOUSE_DIGIN_HPP_

#include <eeros/hal/Input.hpp>
#include <eeros/hal/Mouse.hpp>
#include <fstream>
#include <string>

namespace eeros {
	namespace hal {
		class MouseDigIn : public Input<bool> {
		public:
			MouseDigIn(std::string id, Mouse* m) : Input<bool>(id, nullptr), m(m) { }
			~MouseDigIn() { }
			virtual bool get() {
				if (getId().compare("leftMouseButton") == 0) return m->current.button.left;
				if (getId().compare("middleMouseButton") == 0) return m->current.button.middle;
				if (getId().compare("rightMouseButton") == 0) return m->current.button.right;
				return false;
			}
			
		private:
			Mouse* m;
		};

	};
};

#endif /* ORG_EEROS_MOUSE_DIGIN_HPP_ */
