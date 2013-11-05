#ifndef ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP
#define ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP

#include <eeros/control/Block1i.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/SystemOutput.hpp>

namespace eeros {
	namespace control {

		class BlockOutput : public Block1i {

		public:
			BlockOutput(std::string id);
			virtual void run();

		private:
			hal::HAL& hal;
			hal::SystemOutput<double>* systemOutput;
		};

	};
};

#endif // ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP
