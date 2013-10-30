#ifndef ORG_EEROS_CONTROL_D_HPP_
#define ORG_EEROS_CONTROL_D_HPP_

#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/Block1i1o.hpp>

#include <vector>

namespace eeros {
	namespace control {

		class D: public Block1i1o {
		public:
			D(sigdim_t dim = 1);
			virtual ~D();

			virtual void run();
			
		private:
			bool first;
			std::vector<realSignalDatum> prev;
		};

	};
};
#endif /* ORG_EEROS_CONTROL_D_HPP_ */
