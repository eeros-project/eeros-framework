#ifndef ORG_EEROS_CONTROL_SPACENAVIGATORINPUT_HPP_
#define ORG_EEROS_CONTROL_SPACENAVIGATORINPUT_HPP_

#include <string>
#include <thread>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>
#include <eeros/hal/SpaceNavigator.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::math;
using namespace eeros::hal;

namespace eeros {
	namespace control {

		class SpaceNavigatorInput : public Block1o<Matrix<SPACENAVIGATOR_AXIS_COUNT>> {
		public:
			SpaceNavigatorInput(std::string dev);
			virtual ~SpaceNavigatorInput();
			
			virtual void run();
			virtual void setInitPos(Matrix<SPACENAVIGATOR_AXIS_COUNT> initPos);
			Output<Matrix<SPACENAVIGATOR_ROT_AXIS_COUNT,1,double>>& getRotOut();
			Output<Matrix<SPACENAVIGATOR_BUTTON_COUNT,1,bool>>& getButtonOut();			

		protected:
			Output<Matrix<SPACENAVIGATOR_ROT_AXIS_COUNT,1,double>> rotOut;
			Output<Matrix<SPACENAVIGATOR_BUTTON_COUNT,1,bool>> buttonOut;
			SpaceNavigator sn;
			std::thread* t;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_SPACENAVIGATORINPUT_HPP_ */
