#ifndef ORG_EEROS_MATH_MATRIX_HPP_
#define ORG_EEROS_MATH_MATRIX_HPP_

#include <eeros/core/EEROSException.hpp>

#include <sstream>
#include <cstdlib>
#include <cmath>
#include <stdint.h>

namespace eeros {
	namespace math {
		template < uint8_t N, uint8_t M = 1, typename T = double >
		class Matrix {
		public:
			Matrix() { }
			
			/********** Functions for initializing the matrix **********/
			
			void zero() {
				for(uint8_t i = 0; i < N * M; i++)
					value[i] = 0;
			}
			
			void eye() {
				zero();
				uint8_t j = (N < M) ? N : M;
				for(uint8_t i = 0; i < j; i++)
					(*this)(i, i) = 1;
			}
			
			void rotx(double angle) {
				Matrix<N,M,T>& m = *this;
				if(N == 3 && M == 3) {
					m(0,0) = 1;
					m(1,0) = 0;
					m(2,0) = 0;

					m(0,1) = 0;
					m(1,1) = std::cos(angle);
					m(2,1) = std::sin(angle);

					m(0,2) = 0;
					m(1,2) = -std::sin(angle);
					m(2,2) = std::cos(angle);
				}
				else {
					throw EEROSException("rotx(double) is only implemented for 3x3 matrices");
				}
			}
			
			void roty(double angle) {
				Matrix<N,M,T>& m = *this;
				if(N == 3 && M == 3) {
					m(0,0) = std::cos(angle);
					m(1,0) = 0;
					m(2,0) = -std::sin(angle);

					m(0,1) = 0;
					m(1,1) = 1;
					m(2,1) = 0;

					m(0,2) = std::sin(angle);
					m(1,2) = 0;
					m(2,2) = std::cos(angle);
				}
				else {
					throw EEROSException("roty(double) is only implemented for 3x3 matrices");
				}
			}
			
			void rotz(double angle) {
				Matrix<N,M,T>& m = *this;
				if(N == 3 && M == 3) {
					m(0,0) = std::cos(angle);
					m(1,0) = std::sin(angle);
					m(2,0) = 0;

					m(0,1) = -std::sin(angle);
					m(1,1) = std::cos(angle);
					m(2,1) = 0;

					m(0,2) = 0;
					m(1,2) = 0;
					m(2,2) = 1;
				}
				else {
					throw EEROSException("rotz(double) is only implemented for 3x3 matrices");
				}
			}
			
			/********** Functions for checking the matrix characteristics **********/
			
			bool isOrthogonal() const {
				// TODO
				return false;
			}
			
			bool isSymmetric() const {
				// TODO
				return false;
			}
			
			bool isDiagonal() const {
				// TODO
				return false;
			}
			
			bool isLowerTriangular() const {
				// TODO
				return false;
			}
			
			bool isUpperTriangular() const {
				// TODO
				return false;
			}
			
			bool isInvertible() const {
				// TODO
				return false;
			}
			
			/********** Functions for calculating some characteristics of the matrix **********/
			
			uint32_t rank() const {
				// TODO
				return 0;
				
			}
			
			T det() const {
				// TODO
				return 0;
			}
			
			T trace() const {
				T result = 0;
				uint8_t j = (N < M) ? N : M;
				for(uint8_t i = 0; i < j; i++)
					result += v(i,i);
				return result;
			}
			
			Matrix<M,N,T> transpose() const {
				Matrix<M,N,T> result;
				for(uint8_t n = 0; n < N; n++) {
					for (uint8_t m = 0; m < M; m++) {
						result(m,n) = (*this)(n,m);
					}
				}
				return result;
			}
			
			/********** Operators **********/
			
			T& operator()(uint8_t n, uint8_t m = 0) {
				if(n >= 0 && n < N && m >= 0 && m < M)
					return value[M * n + m];
				else {
					std::stringstream msg;
					msg << "Access to element failed: Index out of bound: n = " << n << ", m = "<< m;
					throw EEROSException(msg.str());
				}
			}

			const T operator()(uint8_t n, uint8_t m = 0) const {
				if (n >= 0 && n < N && m >= 0 && m < M)
					return value[M * n + m];
				else {
					std::stringstream msg;
					msg << "Access to element failed: Index out of bound: n = " << n << ", m = "<< m;
					throw EEROSException(msg.str());
				}
			}

			template < uint8_t K >
			Matrix<N,K,T> operator*(const Matrix<M,K,T> right) const {
				Matrix<N,K,T> result;
				for (uint8_t n = 0; n < N; n++) {
					for (uint8_t k = 0; k < K; k++) {
						result(n, k) = 0;
						for (uint8_t m = 0; m < M; m++) {
							result(n,k) += (*this)(n, m) * right(m, k);
						}
					}
				}
				return result;
			}
			
			Matrix<N,M,T> operator*(T right) const {
				Matrix<N,M,T> result;
				for (uint8_t n = 0; n < N; n++) {
					for (uint8_t m = 0; m < M; m++) {
						result(n,m) = (*this)(n,m) * right;
					}
				}
				return result;
			}
			
			Matrix<N,M,T> operator+(const Matrix<N,M,T> right) const {
				Matrix<N,M,T> result;
				for (uint8_t n = 0; n < N; n++) {
					for (uint8_t m = 0; m < M; m++) {
						result(n,m) = (*this)(n,m) + right(n,m);
					}
				}
				return result;
			}
			
			Matrix<N,M,T> operator+(const T right) const {
				Matrix<N,M,T> result;
				for (uint8_t n = 0; n < N; n++) {
					for (uint8_t m = 0; m < M; m++) {
						result(n,m) = (*this)(n,m) + right;
					}
				}
				return result;
			}

			Matrix<N,M,T> operator-(const Matrix<N,M,T> right) const {
				Matrix<N,M,T> result;
				for (uint8_t n = 0; n < N; n++) {
					for (uint8_t m = 0; m < M; m++) {
						result(n,m) = (*this)(n,m) - right(n,m);
					}
				}
				return result;
			}
			
			Matrix<N,M,T> operator/(T right) const {
				Matrix<N,M,T> result;
				for(uint8_t n = 0; n < N; n++) {
					for(uint8_t m = 0; m < M; m++) {
						result(n,m) = (*this)(n,m) / right;
					}
				}
				return result;
			}

			bool operator==(const Matrix<N,M,T> right) const {
				for(uint8_t n = 0; n < N; n++) {
					for(uint8_t m = 0; m < M; m++) {
						if(v(n,m) == right(n,m))
							return false;
					}
				}
				return true;
			}

			bool operator!=(const Matrix<N,M,T> right) const {
				for(uint8_t n = 0; n < N; n++) {
					for(uint8_t m = 0; m < M; m++) {
						if(v(n,m) != right(n,m))
							return true;
					}
				}
				return false;
			}
			
			Matrix<N,M,T> operator!() const {
				if(N == 3 && M == 3) {
					T det = (
								v(0, 0) * v(1, 1) * v(2, 2) +
								v(0, 1) * v(1, 2) * v(2, 0) +
								v(0, 2) * v(1, 0) * v(2, 1) -
								v(0, 2) * v(1, 1) * v(2, 0) -
								v(0, 0) * v(1, 2) * v(2, 1) -
								v(0, 1) * v(1, 0) * v(2, 2)
							);
					if(det == 0) throw EEROSException("Invert failed: determinat of matrix is 0");
					
					Matrix<N,M,T> result;
					result(0, 0) = v(1, 1) * v(2, 2) - v(1, 2) * v(2, 1);
					result(1, 0) = v(1, 2) * v(2, 0) - v(1, 0) * v(2, 2);
					result(2, 0) = v(1, 0) * v(2, 1) - v(1, 1) * v(2, 0);
					result(0, 1) = v(0, 2) * v(2, 1) - v(0, 1) * v(2, 2);
					result(1, 1) = v(0, 0) * v(2, 2) - v(0, 2) * v(2, 0);
					result(2, 1) = v(0, 1) * v(2, 0) - v(0, 0) * v(2, 1);
					result(0, 2) = v(0, 1) * v(1, 2) - v(0, 2) * v(1, 1);
					result(1, 2) = v(0, 2) * v(1, 0) - v(0, 0) * v(1, 2);
					result(2, 2) = v(0, 0) * v(1, 1) - v(0, 1) * v(1, 0);
					return (result / det);
				}
				else if(N == 2 && M == 2) {
					T det = ( v(0, 0) * v(1, 1) - v(0, 1) * v(1, 0) );
					if(det == 0) throw EEROSException("Invert failed: determinat of matrix is 0");

					Matrix<N,M,T> result;
					result(0, 0) = v(1, 1);
					result(1, 0) = -v(1, 0);
					result(0, 1) = -v(0, 1);
					result(1, 1) = v(0, 0);
					return (result / det);
				}
				else {
					// TODO
					throw EEROSException("Invert failed: function only implemented for matrices with dimension 2x2 and 3x3");
				}
			}

			operator T () const {
				if (N == 1 && M == 1)
					return value[0];
				else 
					throw EEROSException("Dimension is not 1x1");
			}

			/********** Static functions **********/
			
			static Matrix<N,M,T> createRotX(double angle) {
				Matrix<N,M,T> m;
				m.rotx(angle);
				return m;
			}
			
			static Matrix<N,M,T> createRotY(double angle) {
				Matrix<N,M,T> m;
				m.roty(angle);
				return m;
			}
			
			static Matrix<N,M,T> createRotZ(double angle) {
				Matrix<N,M,T> m;
				m.rotz(angle);
				return m;
			}
			
			static Matrix<2,1,T> createVector2(T x, T y) {
				Matrix<2,1,T> v;
				v(0) = x;
				v(1) = y;
				return v;
			}
			
			static Matrix<3,1,T> createVector3(T x, T y, T z) {
				Matrix<3,1,T> v;
				v(0) = x;
				v(1) = y;
				v(2) = z;
				return v;
			}
			
		private:
			T value[ N * M ];
			
			T& v(uint8_t n, uint8_t m = 0) {
				if (n >= 0 && n < N && m >= 0 && m < M)
					return value[M * n + m ];
				else {
					std::stringstream msg;
					msg << "v(): Index out of bound: n = " << n << ", m = "<< m;
					throw EEROSException(msg.str());
				}
			}
			
			const T& v(uint8_t n, uint8_t m = 0) const {
				if (n >= 0 && n < N && m >= 0 && m < M)
					return value[M * n + m];
				else {
					std::stringstream msg;
					msg << "v(): Index out of bound: n = " << n << ", m = "<< m;
					throw EEROSException(msg.str());
				}
			}
		}; // END class Matrix
		
		/********** Operators **********/
		
		template < uint8_t N, uint8_t M = 1, typename T = double >
		Matrix<N,M,T> operator*(T left, Matrix<N,M,T> right) {
			Matrix<N,M,T> result;
			for (uint8_t n = 0; n < N; n++) {
				for (uint8_t m = 0; m < M; m++) {
					result(n,m) = right(n,m) * left;
				}
			}
			return result;
		}
		
		typedef Matrix<2,1> Vector2;
		typedef Matrix<3,1> Vector3;
		typedef Matrix<4,1> Vector4;
		
	} // END namespace math
} // END namespache eeros

#endif /* ORG_EEROS_MATH_MATRIX_HPP_ */
