#include <eeros/control/Constant.hpp>

using namespace eeros::control;

Constant::Constant(double value, sigdim_t dim) : Block1o(dim), val(dim) {
	for(sigdim_t i = 0; i < dim; i++) {
		this->val[i] = value;
	}
}

Constant::Constant(std::vector<double> values, sigdim_t dim) : Block1o(dim), val(dim) {
	if(values.size() == dim) val = values;
}

Constant::~Constant() {
	// nothing to do
}

void Constant::run() {
	for(int i = 0; i < out.getDimension(); i++) {
		out.setValue(val[i], i);
		out.setTimeStamp(System::getTimeNs(), i);
	}
}

void Constant::setValue(double value) {
	for(int i = 0; i < val.size(); i++) {
		val[i] = value;
	}
}

void Constant::setValue(std::vector<double> values) {
	if(values.size() == out.getDimension()) {
		for(int i = 0; i < val.size(); i++) {
			val[i] = values[i];
		}
	}
}
