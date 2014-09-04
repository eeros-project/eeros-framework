#ifndef ORG_EEROS_CONTROL_VECTOR2CORRECTOR_HPP_
#define ORG_EEROS_CONTROL_VECTOR2CORRECTOR_HPP_

#include <eeros/math/Matrix.hpp>
#include <vector>

namespace eeros {
	namespace control {
		
		class Vector2Corrector {
		public:
			virtual bool load(const char *filename);
			
			virtual eeros::math::Vector2 get(const eeros::math::Vector2 &in);
			
		protected:
			virtual int find_interval(double value, std::vector<double>& list);
			virtual eeros::math::Vector2 get_mapped(int ix, int iy);
			
			std::vector<double> x_ref;
			std::vector<double> y_ref;
			std::vector<eeros::math::Vector2> mapped;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_VECTOR2CORRECTOR_HPP_ */