#ifndef ORG_EEROS_CONTROL_XBOXINPUT_HPP_
#define ORG_EEROS_CONTROL_XBOXINPUT_HPP_

#include <string>
#include <thread>
#include <eeros/control/Output.hpp>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>
#include <eeros/hal/Joystick.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
	namespace control {

		class XBoxInput: public eeros::control::Block1o<eeros::math::Vector4> {
		public:
			XBoxInput(std::string dev);
			virtual ~XBoxInput();
			
			virtual void run();
			virtual void setInitPos(eeros::math::Vector4 initPos);
			virtual void setSpeedScaleFactor(double speedScale);
			
			double speedScaleFactor = 1.0;

		protected:
			eeros::math::Matrix<4,4,double> axisScale;
			
			static constexpr double xScale = 0.0001;
			static constexpr double yScale = 0.0001;
			static constexpr double zScale = 0.0001;
			static constexpr double rScale = 0.001;
			
			eeros::hal::Joystick j;
			std::thread* t;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_XBOXINPUT_HPP_ */
