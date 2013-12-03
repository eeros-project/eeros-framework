#ifndef ORG_EEROS_CONTROL_CONSTANT_HPP_
#define ORG_EEROS_CONTROL_CONSTANT_HPP_

#include <vector>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
	namespace control {

		class Constant: public Block1o {
		public:
			Constant(double value = 0.0, sigdim_t dim = 1);
			Constant(const std::vector<double> values, sigdim_t dim);
			virtual ~Constant();

			virtual void run();
			
			virtual void setValue(double value);
			virtual void setValue(std::vector<double> values);
			
		protected:
			std::vector<double> val;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_CONSTANT_HPP_ */
