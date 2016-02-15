#ifndef ORG_EEROS_CORE_STATISTICS_HPP_
#define ORG_EEROS_CORE_STATISTICS_HPP_

namespace eeros {
	
	class Statistics {
	public:
		Statistics();
		void add(double value);
		void reset();

		long count;
		double last;
		double min;
		double max;
		double mean;
		double variance;

	private:
		double A, B;
	};
};

#endif // ORG_EEROS_CORE_STATISTICS_HPP_
