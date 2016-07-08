#ifndef CH_NTB_PARALLELSCARA_FILTER_HPP_
#define CH_NTB_PARALLELSCARA_FILTER_HPP_

#include <eeros/control/Block1i1o.hpp>
#include "../types.hpp"
#include "../constants.hpp"

#include <stdlib.h>
#include <cmath>

namespace parallelscara {
		class Filter: public eeros::control::Block1i1o<eeros::math::Vector2> {
			
		public:
			Filter(double k);
			virtual ~Filter();
			virtual void run();
			
		protected:
			double k;
			AxisVector prev;
		};
	};
#endif /* CH_NTB_PARALLELSCARA_FILTER_HPP_ */