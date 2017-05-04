#ifndef ORG_EEROS_CONTROL_KEYBOARDINPUT_HPP_
#define ORG_EEROS_CONTROL_KEYBOARDINPUT_HPP_

#include <string>
#include <thread>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>
#include <eeros/hal/Keyboard.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::math;
using namespace eeros::hal;

namespace eeros {
	namespace control {

		class KeyboardInput: public Block1o<Vector4> {
		public:
			KeyboardInput();
			virtual ~KeyboardInput();

			Output<Vector<5,bool>>& getIsHomed();
			Output<bool>& getEsc();
			Output<bool>& getEmergency();
			Output<bool>& getReset();
			Output<bool>& getStart();
			Output<bool>& getStop();
			virtual void run();
			
			Keyboard k;

		protected:
			Output<Vector<5,bool>> isHomed;
			Output<bool> esc;
			Output<bool> emergency;
			Output<bool> reset;
			Output<bool> start;
			Output<bool> stop;
			std::thread* t;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_KEYBOARDINPUT_HPP_ */
