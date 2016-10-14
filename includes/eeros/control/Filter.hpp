#ifndef ORG_EEROS_CONTROL_FILTER_HPP_
#define ORG_EEROS_CONTROL_FILTER_HPP_

#include <eeros/control/Block1i1o.hpp>
#include "../types.hpp"
#include "../constants.hpp"

#include <stdlib.h>
#include <cmath>

namespace eeros {
	namespace control {
		
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
};

#endif /* ORG_EEROS_CONTROL_FILTER_HPP_ */