#ifndef ORG_EEROS_CORE_MATH_MATRIX_HPP_
#define ORG_EEROS_CORE_MATH_MATRIX_HPP_

#include <cstdlib>
#include <cmath>
#include <stdint.h>

namespace eeros {
	namespace math {
		template < uint8_t N, uint8_t M = 1, typename T = double >
		class Matrix {
		public:
			Matrix() { }

			void zero() {
				for (uint8_t i = 0; i < N*M; i++)
					value[i] = 0;
			}

			void eye() {
				zero();
				uint8_t j = (N < M) ? N : M;
				for (uint8_t i = 1; i <= j; i++)
					(*this)(i, i) = 1;
			}

			T trace() const {
				T result = 0;
				uint8_t j = (N < M) ? N : M;
				for (uint8_t i = 1; i <= j; i++)
					result += v(i,i);
				return result;
			}

			T norm() const {
				if (N != 1 && M != 1)
					throw; // TODO throw exception

				T result = 0;
				for (uint8_t i = 0; i < N*M; i++) {
					result += value[i];
				}
				return sqrt(result);
			}

			Matrix<M,N,T> transpose() const {
				Matrix<M,N,T> result;
				for (uint8_t n = 1; n <= N; n++) {
					for (uint8_t m = 1; m <= M; m++) {
						result(m,n) = (*this)(n,m);
					}
				}
				return result;
			}

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

			static Matrix<3,1,T> createVector3(double x, double y, double z) {
				Matrix<3,1,T> v;
				v(1) = x;
				v(2) = y;
				v(3) = z;
				return v;
			}

			/** Makes the matrix a rotation matrix around the x axis by the specified angle.
			 *
			 * This method only applies to 3x3 matrices.
			 *
			 * @param angle angle of the rotation around the x axis
			 */

			void rotx(double angle) {
				Matrix<N,M,T>& m = *this;
				if (N == 3 && M == 3) {
					m(1,1) = 1;
					m(2,1) = 0;
					m(3,1) = 0;

					m(1,2) = 0;
					m(2,2) = std::cos(angle);
					m(3,2) = std::sin(angle);

					m(1,3) = 0;
					m(2,3) = -std::sin(angle);
					m(3,3) = std::cos(angle);
				}
				else {
					throw; // TODO throw exception
				}
			}

			/** Makes the matrix a rotation matrix around the y axis by the specified angle.
			 *
			 * This method only applies to 3x3 matrices.
			 *
			 * @param angle angle of the rotation around the y axis
			 */
			void roty(double angle) {
				Matrix<N,M,T>& m = *this;
				if (N == 3 && M == 3) {
					m(1,1) = std::cos(angle);
					m(2,1) = 0;
					m(3,1) = -std::sin(angle);

					m(1,2) = 0;
					m(2,2) = 1;
					m(3,2) = 0;

					m(1,3) = std::sin(angle);
					m(2,3) = 0;
					m(3,3) = std::cos(angle);
				}
				else {
					throw; // TODO throw exception
				}
			}

			/** Makes the matrix a rotation matrix around the z axis by the specified angle.
			 *
			 * This method only applies to 3x3 matrices.
			 *
			 * @param angle angle of the rotation around the z axis
			 */
			void rotz(double angle) {
				Matrix<N,M,T>& m = *this;
				if (N == 3 && M == 3) {
					m(1,1) = std::cos(angle);
					m(2,1) = std::sin(angle);
					m(3,1) = 0;

					m(1,2) = -std::sin(angle);
					m(2,2) = std::cos(angle);
					m(3,2) = 0;

					m(1,3) = 0;
					m(2,3) = 0;
					m(3,3) = 1;
				}
				else {
					throw; // TODO throw exception
				}
			}

			bool invert(const Matrix<N,M,T> &source) const {
				// TODO
				return false;
			}

			T& operator()(uint8_t n, uint8_t m = 1) {
				if (n >= 1 && n <= N && m >= 1 && m <= M)
					return value[ M*(n-1) + (m-1) ];
				else
					throw; // TODO throw exception
			}

			const T operator()(uint8_t n, uint8_t m = 1) const {
				if (n >= 1 && n <= N && m >= 1 && m <= M)
					return value[ M*(n-1) + (m-1) ];
				else
					throw; // TODO throw exception
			}

			template < uint8_t K >
			Matrix<N,K,T> operator*(const Matrix<M,K,T> right) const {
				Matrix<N,K,T> result;

				for (uint8_t n = 1; n <= N; n++) {
					for (uint8_t k = 1; k <= K; k++) {
						result(n, k) = 0;
						for (uint8_t m = 1; m <= M; m++) {
							result(n, k) += (*this)(n, m) * right(m, k);
						}
					}
				}

				return result;
			}

			Matrix<N,M,T> operator+(const Matrix<N,M,T> right) const {
				Matrix<N,M,T> result;

				for (uint8_t n = 1; n <= N; n++) {
					for (uint8_t m = 1; m <= M; m++) {
						result(n, m) = (*this)(n,m) + right(n,m);
					}
				}

				return result;
			}

			Matrix<N,M,T> operator-(const Matrix<N,M,T> right) const {
				Matrix<N,M,T> result;

				for (uint8_t n = 1; n <= N; n++) {
					for (uint8_t m = 1; m <= M; m++) {
						result(n, m) = (*this)(n,m) - right(n,m);
					}
				}

				return result;
			}

			Matrix<N,M,T> operator*(T right) const {
				Matrix<N,M,T> result;

				for (uint8_t n = 1; n <= N; n++) {
					for (uint8_t m = 1; m <= M; m++) {
						result(n, m) = (*this)(n,m) * right;
					}
				}

				return result;
			}

			Matrix<N,M,T> operator/(T right) const {
				Matrix<N,M,T> result;

				for (uint8_t n = 1; n <= N; n++) {
					for (uint8_t m = 1; m <= M; m++) {
						result(n, m) = (*this)(n,m) / right;
					}
				}

				return result;
			}

			bool operator==(const Matrix<N,M,T> right) const {
				for (uint8_t n = 1; n <= N; n++) {
					for (uint8_t m = 1; m <= M; m++) {
						if (v(n,m) == right(n,m))
							return false;
					}
				}

				return true;
			}

			bool operator!=(const Matrix<N,M,T> right) const {
				for (uint8_t n = 1; n <= N; n++) {
					for (uint8_t m = 1; m <= M; m++) {
						if (v(n,m) != right(n,m))
							return true;
					}
				}

				return false;
			}


			Matrix<N,M,T> operator!() const {
				if (N == 3 && M == 3) {
					T det = (
								v(1, 1) * v(2, 2) * v(3, 3) +
								v(1, 2) * v(2, 3) * v(3, 1) +
								v(1, 3) * v(2, 1) * v(3, 2) -
								v(1, 3) * v(2, 2) * v(3, 1) -
								v(1, 2) * v(2, 1) * v(3, 3) -
								v(1, 1) * v(2, 3) * v(3, 2)
							);

					if (det == 0) throw; // TODO throw exception

					Matrix<N,M,T> result;

					result(1, 1) = v(2, 2) * v(3, 3) - v(2, 3) * v(3, 2);
					result(2, 1) = v(2, 3) * v(3, 1) - v(2, 1) * v(3, 3);
					result(3, 1) = v(2, 1) * v(3, 2) - v(2, 2) * v(3, 1);

					result(1, 2) = v(1, 3) * v(3, 2) - v(1, 2) * v(3, 3);
					result(2, 2) = v(1, 1) * v(3, 3) - v(1, 3) * v(3, 1);
					result(3, 2) = v(1, 2) * v(3, 1) - v(1, 1) * v(3, 2);

					result(1, 3) = v(1, 2) * v(2, 3) - v(1, 3) * v(2, 2);
					result(2, 3) = v(1, 3) * v(2, 1) - v(1, 1) * v(2, 3);
					result(3, 3) = v(1, 1) * v(2, 2) - v(1, 2) * v(2, 1);

					return (result / det);
				}
				else if (N == 2 && M == 2) {
					T det = ( v(1, 1) * v(2, 2) - v(1, 2) * v(2, 1) );
					if (det == 0) throw; // TODO throw exception

					Matrix<N,M,T> result;

					result(1, 1) = v(2, 2);
					result(2, 1) = -v(2, 1);

					result(1, 2) = -v(1, 2);
					result(2, 2) = v(1, 1);

					return (result / det);
				}
				else {
					throw; // TODO throw exception
				}
			}

			operator T () const {
				if (N == 1 && M == 1)
					return value[0];
				else
					throw; // TODO throw exception
			}

		private:
			T value[ N * M ];
			T& v(uint8_t n, uint8_t m = 1) {
				if (n >= 1 && n <= N && m >= 1 && m <= M)
					return value[ M*(n-1) + (m-1) ];
				else
					throw; // TODO throw exception
			}
			const T& v(uint8_t n, uint8_t m = 1) const {
				if (n >= 1 && n <= N && m >= 1 && m <= M)
					return value[ M*(n-1) + (m-1) ];
				else
					throw; // TODO throw exception
			}
		};

		typedef Matrix<2,1> Vector2;
		typedef Matrix<3,1> Vector3;
		typedef Matrix<4,1> Vector4;
	}
}

#endif /* ORG_EEROS_CORE_MATH_MATRIX_HPP_ */
