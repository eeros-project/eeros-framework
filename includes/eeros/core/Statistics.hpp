#ifndef ORG_EEROS_CORE_STATISTICS_HPP_
#define ORG_EEROS_CORE_STATISTICS_HPP_

namespace eeros {
	
	class Statistics {
	public:
		Statistics();
		Statistics(long max_count);
		void add(double value);
		void reset();
		
		long max_count;
		long count;
		double min;
		double max;
		double mean;
	};
};

#endif // ORG_EEROS_CORE_STATISTICS_HPP_
