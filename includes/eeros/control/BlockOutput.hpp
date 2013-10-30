#ifndef ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP
#define ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP

#include <eeros/control/Block1i.hpp>

namespace eeros {
	namespace control {

		struct signalData {
			double value;
			uint64_t timestamp;
		};

		class BlockOutput : public Block1i {

		public:
			BlockOutput();
			virtual ~BlockOutput();
			virtual void run();

		private:
			std::string identifier;
			double scale;
			double offset;
			double min;
			double max;
			
			signalData dat;
		};

	};
};

#endif // ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP
