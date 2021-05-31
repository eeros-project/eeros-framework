#ifndef ORG_EEROS_CONTROL_SPACENAVIGATORINPUT_HPP_
#define ORG_EEROS_CONTROL_SPACENAVIGATORINPUT_HPP_

#include <string>
#include <thread>
#include <eeros/control/Blockio.hpp>
#include <eeros/core/System.hpp>
#include <eeros/hal/SpaceNavigator.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::math;
using namespace eeros::hal;

namespace eeros {
	namespace control {

		class SpaceNavigatorInput : public Blockio<0,1,Matrix<SPACENAVIGATOR_AXIS_COUNT>> {
		public:
			SpaceNavigatorInput(std::string dev, int priority = 5);
			virtual ~SpaceNavigatorInput();
			
			virtual void run();
			virtual void setInitPos(Matrix<SPACENAVIGATOR_AXIS_COUNT> initPos);
			Output<Matrix<SPACENAVIGATOR_ROT_AXIS_COUNT,1,double>>& getRotOut();
			Output<Matrix<SPACENAVIGATOR_BUTTON_COUNT,1,bool>>& getButtonOut();			

		protected:
			SpaceNavigator sn;
			Output<Matrix<SPACENAVIGATOR_ROT_AXIS_COUNT,1,double>> rotOut;
			Output<Matrix<SPACENAVIGATOR_BUTTON_COUNT,1,bool>> buttonOut;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_SPACENAVIGATORINPUT_HPP_ */
