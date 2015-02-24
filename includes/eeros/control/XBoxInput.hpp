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
			
			inline void on_button(std::function<void(int, bool)> &&action)
			{
				j.on_button(std::move(action));
			}
			
			double speedScaleFactor = 1.0;

			double xScale = 0.0001;
			double yScale = 0.0001;
			double zScale = 0.0001;
			double rScale = 0.002;
			double min_x = -0.03;
			double max_x = 0.045;
			double min_y = -0.03;
			double max_y = 0.03;
			double min_z = -0.053;
			double max_z = -0.015;
			double min_r = -2.8;
			double max_r = 2.8;

		protected:
			eeros::math::Matrix<4,4,double> axisScale;
			
			eeros::hal::Joystick j;
			std::thread* t;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_XBOXINPUT_HPP_ */
