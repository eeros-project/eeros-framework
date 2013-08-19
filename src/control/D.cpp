#include <eeros/control/D.hpp>

#include <iostream>

D::D(sigdim_t dim) : Block1i1o(dim), prev(dim) {
	first = true;
}

D::~D() {
	// nothing to do...
}

void D::run() {
	if(first) { // first run, no previous value available -> set output to zero
		for(int i = 0; i < out.getDimension(); i++) {
			out.setValue(0, i);
			out.setTimeStamp(in.getTimestamp(i), i);
			prev[i].value = in.getValue(i);
			prev[i].timestamp = in.getTimestamp(i);
		}
		first = false;
	}
	else {
		for(int i = 0; i < out.getDimension(); i++) {
			out.setValue((in.getValue(i) - prev[i].value) / ((in.getTimestamp(i) - prev[i].timestamp) / 1000000000.0), i);
			out.setTimeStamp((in.getTimestamp(i) + prev[i].timestamp) / 2, i);
			prev[i].value = in.getValue(i);
			prev[i].timestamp = in.getTimestamp(i);
		}
	}
}