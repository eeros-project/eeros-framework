#ifndef ORG_EEROS_CONTROL_MOUSEINPUT_HPP_
#define ORG_EEROS_CONTROL_MOUSEINPUT_HPP_

#include <string>
#include <thread>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>
#include <eeros/hal/Mouse.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::math;
using namespace eeros::hal;

namespace eeros {
	namespace control {

		class MouseInput: public Block1o<Vector4> {
		public:
			MouseInput(std::string dev);
			virtual ~MouseInput();

			Output<Matrix<3,1,bool>>& getButtonOut();
			virtual void run();
			virtual void setInitPos(double x, double y, double z, double r);
			virtual void setInitPos(Vector4 pos);
			virtual void reset(double x, double y, double z, double r);

			Mouse j;
			double axisScale_x = 0.0001;
			double axisScale_y = 0.0001;
			double axisScale_z = 0.001;
			double axisScale_r = 0.2;
			double min_x = -0.03;
			double max_x = 0.045;
			double min_y = -0.03;
			double max_y = 0.03;
			double min_z = -0.053;
			double max_z = -0.015;
			double min_r = -2.8;
			double max_r = 2.8;

		protected:
			Output<Matrix<3,1,bool>> buttonOut;
			double x, y, z, r;
			bool first;
			std::thread* t;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_MOUSEINPUT_HPP_ */
