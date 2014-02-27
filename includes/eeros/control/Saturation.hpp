#ifndef ORG_EEROS_CONTROL_SATURATION_HPP_
#define ORG_EEROS_CONTROL_SATURATION_HPP_

#include <vector>

#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {

		class Saturation: public Block1i1o {
		public:
			Saturation(double lim, sigdim_t dim = 1);
			Saturation(double lowLimit, double upLimit, sigdim_t dim = 1);
			Saturation(const double lim[], sigdim_t dim);
			Saturation(const double lowLimit[], const double upLimit[], sigdim_t dim);
			Saturation(const std::vector<double>, sigdim_t dim);
			Saturation(const std::vector<double> lowLimit, const std::vector<double> upLimit, sigdim_t dim);
			virtual ~Saturation();

			virtual void run();
			
			virtual void enable();
			virtual void disable();
			virtual void setSaturationLimit(double lim);
			virtual void setSaturationLimit(double lowLim, double upLim);
			virtual void setSaturationLimit(sigindex_t index, double lim);
			virtual void setSaturationLimit(sigindex_t index, double lowLim, double upLimit);
			
		protected:
			std::vector<double> upLimit;
			std::vector<double> lowLimit;
			bool enabled;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_SATURATION_HPP_ */
