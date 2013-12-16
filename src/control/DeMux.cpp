#include <eeros/control/DeMux.hpp>

namespace eeros {
	namespace control {
		
		DeMux::DeMux(sigdim_t inputDimension) : out(inputDimension), in() { }
			
		void DeMux::run() {
			for(sigdim_t i = 0; i < in.getDimension(); i++) {
				out[i].setValue(in.getValue(i));
				out[i].setTimeStamp(in.getTimestamp(i));
			}
		}
		
		RealSignalInput& DeMux::getIn() {
			return in;
		}

		RealSignalOutput& DeMux::getOut(uint8_t output) {
			return out[output];
		}
		
	};
};