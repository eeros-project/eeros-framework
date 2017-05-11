#ifndef ORG_EEROS_HAL_SPACENAVIGATOR_HPP_
#define ORG_EEROS_HAL_SPACENAVIGATOR_HPP_

#include <string>
#include <functional>
#include <eeros/hal/Input.hpp>

#define SPACENAVIGATOR_AXIS_COUNT (3)
#define SPACENAVIGATOR_ROT_AXIS_COUNT (3)
#define SPACENAVIGATOR_BUTTON_COUNT (2)

namespace eeros {
	namespace hal {
		struct SpaceState {
			double axis[SPACENAVIGATOR_AXIS_COUNT];
			double rotAxis[SPACENAVIGATOR_ROT_AXIS_COUNT];
			bool button[SPACENAVIGATOR_BUTTON_COUNT];
		};
		
		struct SpaceNav {
			struct Axis {
				static constexpr int X = 0;
				static constexpr int Y = 1;
				static constexpr int Z = 2;
			};
			struct RotAxis {
				static constexpr int RX = 0;
				static constexpr int RY = 1;
				static constexpr int RZ = 2;
			};
			struct Button {
				static constexpr int L = 0;
				static constexpr int R = 1;
			};
		};
		
		class SpaceNavigator {
		public:
			explicit SpaceNavigator();
			~SpaceNavigator();
			virtual bool open(const char* device);
			virtual void close();
			virtual void loop();			
			virtual std::string name();
			
			SpaceState current;
			
		private:
			FILE* file;
			Input<bool>* button[SPACENAVIGATOR_BUTTON_COUNT];
		};
	}
}

#endif // ORG_EEROS_HAL_SPACENAVIGATOR_HPP_
