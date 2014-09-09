#ifndef ORG_EEROS_CONTROL_VECTOR2CORRECTOR_HPP_
#define ORG_EEROS_CONTROL_VECTOR2CORRECTOR_HPP_

#include <eeros/math/Matrix.hpp>
#include <vector>

namespace eeros {
	namespace control {
		
		class Vector2Corrector {
		public:
			virtual bool load(const char *filename);
			
			virtual int count();
			virtual eeros::math::Vector2 get(const eeros::math::Vector2 &in);
			
		protected:
			struct map {
				eeros::math::Vector2 Aref;			// triangle origin (of reference system)
				eeros::math::Matrix<2,2> Tiref;		// transformation: cartesian to triangle coordinates (of reference system)
				eeros::math::Vector2 Amapped;		// triangle origin (of mapped system)
				eeros::math::Matrix<2,2> Tmapped;	// transformation: triangle to cartesian coordinates (of mapped system)
			};
			std::vector<map> maps;
			std::vector<eeros::math::Vector2> mapped; // preallocated buffer
		};

	};
};

#endif /* ORG_EEROS_CONTROL_VECTOR2CORRECTOR_HPP_ */