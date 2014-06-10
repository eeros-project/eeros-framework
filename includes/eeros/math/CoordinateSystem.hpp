#ifndef ORG_EEROS_MATH_COORDINATESYSTEM_HPP_
#define ORG_EEROS_MATH_COORDINATESYSTEM_HPP_

#include <string>

namespace eeros {
	namespace math {
		
		class CoordinateSystem {
		public:
			CoordinateSystem(std::string id);
			
			bool operator==(const CoordinateSystem& right) const;
			bool operator!=(const CoordinateSystem& right) const;
		
		private:
			CoordinateSystem(const CoordinateSystem&);
			CoordinateSystem& operator=(const CoordinateSystem&);
			
			std::string id;
		
		}; // END class CoordinateSystem
	} // END namespace math
} // END namespache eeros

#endif /* ORG_EEROS_MATH_COORDINATESYSTEM_HPP_ */
