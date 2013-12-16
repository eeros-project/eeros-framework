#include <eeros/control/Mux.hpp>

namespace eeros {
	namespace control {
		
		Mux::Mux(sigdim_t nofInputs) : out(nofInputs), in(nofInputs) { }
			
		void Mux::run() {
			for(sigdim_t i = 0; i < out.getDimension(); i++) {
				out.setValue(in[i].getValue(0), i);
				out.setTimeStamp(in[i].getTimestamp(0), i);
			}
		}
		
		RealSignalInput& Mux::getIn(uint8_t input) {
			return in[input];
		}

		RealSignalOutput& Mux::getOut() {
			return out;
		}
		
	};
};