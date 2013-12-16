#ifndef ORG_EEROS_CONTROL_DEMUX_HPP_
#define ORG_EEROS_CONTROL_DEMUX_HPP_

#include <vector>

#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>

namespace eeros {
	namespace control {

		class DeMux: public Block {
		public:
			DeMux(sigdim_t inputDimension);
			
			virtual void run();
			
			virtual RealSignalInput& getIn();
			virtual RealSignalOutput& getOut(uint8_t output = 0);
			
		protected:
			RealSignalInput in;
			std::vector<RealSignalOutput> out;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_DEMUX_HPP_ */
