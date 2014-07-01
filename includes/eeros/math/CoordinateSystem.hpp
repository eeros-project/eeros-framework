#ifndef ORG_EEROS_MATH_COORDINATESYSTEM_HPP_
#define ORG_EEROS_MATH_COORDINATESYSTEM_HPP_

#include <string>
#include <map>
#include <ostream>

namespace eeros {
	namespace math {
		
		class CoordinateSystem {
			
			friend std::ostream& operator<<(std::ostream& os, const CoordinateSystem& cs);
			
		public:
			CoordinateSystem(std::string id);
			virtual ~CoordinateSystem();
			
			bool operator==(const CoordinateSystem& right) const;
			bool operator!=(const CoordinateSystem& right) const;
			
			
			static CoordinateSystem* getCoordinateSystem(std::string id);
			
		private:
			CoordinateSystem(const CoordinateSystem&);
			CoordinateSystem& operator=(const CoordinateSystem&);
			
			std::string id;
			
			static std::map<std::string, CoordinateSystem*> list;
			
		
		}; // END class CoordinateSystem
		
		std::ostream& operator<<(std::ostream& os, const CoordinateSystem& cs);
		
	} // END namespace math
} // END namespache eeros

#endif /* ORG_EEROS_MATH_COORDINATESYSTEM_HPP_ */
