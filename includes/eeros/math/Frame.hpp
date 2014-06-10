#ifndef ORG_EEROS_MATH_FRAME_HPP_
#define ORG_EEROS_MATH_FRAME_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/math/CoordinateSystem.hpp>

namespace eeros {
	namespace math {
		
		class Frame {
		public:
			Frame(const CoordinateSystem& a, const CoordinateSystem& b);
			Frame(const CoordinateSystem& a, const CoordinateSystem& b, eeros::math::Matrix<4, 4, double> T);
			Frame(const CoordinateSystem& a, const CoordinateSystem& b, eeros::math::Matrix<3, 3, double> R, eeros::math::Matrix<3, 1, double> r);
			
			Frame operator*(const Frame& right) const;
			
			void set(eeros::math::Matrix<4, 4, double> T);
			void set(eeros::math::Matrix<3, 3, double> R, eeros::math::Matrix<3, 1, double> r);
			eeros::math::Matrix<4, 4, double> get() const;
			const CoordinateSystem& getFromCoordinateSystem() const;
			const CoordinateSystem& getToCoordinateSystem() const;
			
		private:
			const CoordinateSystem& a;
			const CoordinateSystem& b;
			eeros::math::Matrix<4, 4, double> T;
		
		}; // END class Frame
	} // END namespace math
} // END namespache eeros

#endif /* ORG_EEROS_MATH_FRAME_HPP_ */
