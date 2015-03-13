#include <eeros/core/Statistics.hpp>
#include <limits>

using namespace eeros;



Statistics::Statistics() : max_count(1000) { }

Statistics::Statistics(long max_count) : max_count(max_count) { }

void Statistics::add(double value)
{
	if (value < min) min = value;
	if (value > max) max = value;
	
	if (count < max_count) {
		mean = (mean * count + value) / (count + 1);
	}
	else {
		mean = (mean * (max_count - 1) + value) / max_count;
	}
	count++;
}

void Statistics::reset()
{
		count = 0;
		min = std::numeric_limits<double>::max();
		max = std::numeric_limits<double>::lowest();
		mean = 0;
}
