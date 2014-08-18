#ifndef ORG_EEROS_MATH_FRAME_HPP_
#define ORG_EEROS_MATH_FRAME_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/math/CoordinateSystem.hpp>
#include <list>

namespace eeros {
	namespace math {
		
		class Frame {
		public:
			Frame(const CoordinateSystem& a, const CoordinateSystem& b);
			Frame(const CoordinateSystem& a, const CoordinateSystem& b, const eeros::math::Matrix<4, 4, double>& T);
			Frame(const CoordinateSystem& a, const CoordinateSystem& b, const eeros::math::Matrix<3, 3, double>& R, const eeros::math::Matrix<3, 1, double>& r);
			virtual ~Frame();
			
			Frame operator*(const Frame& right) const;
			
			void set(const eeros::math::Matrix<4, 4, double>& T);
			void set(const eeros::math::Matrix<3, 3, double>& R, const eeros::math::Matrix<3, 1, double>& r);
			eeros::math::Matrix<4, 4, double> get() const;
			const CoordinateSystem& getFromCoordinateSystem() const;
			const CoordinateSystem& getToCoordinateSystem() const;
			
			static Frame* getFrame(const CoordinateSystem& a, const CoordinateSystem& b);
			static uint32_t getNofFrames();
			
		private:
			const CoordinateSystem& a;
			const CoordinateSystem& b;
			eeros::math::Matrix<4, 4, double> T;
			
			static std::list<Frame*> list;
		
		}; // END class Frame
	} // END namespace math
} // END namespache eeros

#endif /* ORG_EEROS_MATH_FRAME_HPP_ */
