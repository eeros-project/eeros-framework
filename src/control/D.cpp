#include <eeros/control/D.hpp>

D::D(sigdim_t dim) : Block1i1o(dim), prev(dim) {
	first = true;
}

D::~D() {
	// nothing to do...
}

void D::run() {
	if(first) { // first run, no previous value available
		for(int i = 0; i < out.getDimension(); i++) {
			out.setValue(0, i);
			out.setTimeStamp(in.getTimestamp());
			prev[i].value = in.getValue(i);
			prev[i].timestamp = in.getTimestamp(i);
		}
		first = false;
	}
	else {
		for(int i = 0; i < out.getDimension(); i++) {
			out.setValue((in.getValue(i) - prev[i].value) / (in.getTimestamp(i) - prev[i].timestamp), i);
			out.setTimeStamp((in.getTimestamp() + prev[i].timestamp) / 2);
			prev[i].value = in.getValue(i);
			prev[i].timestamp = in.getTimestamp(i);
		}
	}
}