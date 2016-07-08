#include "Filter.hpp"

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;
using namespace parallelscara;

Filter::Filter(double k) : k(k){ }

Filter::~Filter() { 
	// nothing to do...
}

void Filter::run() {
	AxisVector filtered; 
	
	filtered = prev * (1.0 - k) + in.getSignal().getValue() * k;
	prev = filtered;
	
	out.getSignal().setValue(filtered);
	out.getSignal().setTimestamp(in.getSignal().getTimestamp());
}
