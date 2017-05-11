#ifndef ORG_EEROS_SPACENAVIGATOR_DIGIN_HPP_
#define ORG_EEROS_SPACENAVIGATOR_DIGIN_HPP_

#include <eeros/hal/Input.hpp>
#include <eeros/hal/SpaceNavigator.hpp>
#include <fstream>
#include <string>

namespace eeros {
	namespace hal {
		class SpaceNavigatorDigIn : public Input<bool> {
		public:
			SpaceNavigatorDigIn(std::string id, SpaceNavigator* sn) : Input<bool>(id, nullptr), sn(sn) { }
			~SpaceNavigatorDigIn() { }
			virtual bool get() {
				if (getId().compare("SpaceNavButtonL") == 0) return sn->current.button[SpaceNav::Button::L];
				if (getId().compare("SpaceNavButtonR") == 0) return sn->current.button[SpaceNav::Button::R];
				return false;
			}
			
		private:
			SpaceNavigator* sn;
		};

	};
};

#endif /* ORG_EEROS_SPACENAVIGATOR_DIGIN_HPP_ */
