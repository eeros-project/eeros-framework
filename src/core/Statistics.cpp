#include <eeros/core/Statistics.hpp>
#include <limits>

using namespace eeros;


Statistics::Statistics() {
	reset();
}

void Statistics::add(double value) {
	if (value < min) min = value;
	if (value > max) max = value;

	count++;
	last = value;

	A += value;
	B += value * value;

	mean = A / count;
	variance = (B - 2*mean*A) / count + mean*mean;
}

void Statistics::reset() {
		count = 0;
		last = 0;
		min = std::numeric_limits<double>::max();
		max = std::numeric_limits<double>::lowest();
		mean = 0;
		variance = 0;
		A = 0;
		B = 0;
}
