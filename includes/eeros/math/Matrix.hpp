#ifndef ORG_EEROS_MATH_MATRIX_HPP_
#define ORG_EEROS_MATH_MATRIX_HPP_

#include <eeros/core/EEROSException.hpp>
#include "MatrixIndexOutOfBoundException.hpp"

#include <sstream>
#include <cstdlib>
#include <cmath>
#include <stdint.h>

namespace eeros {
	namespace math {
		template < uint8_t M, uint8_t N = 1, typename T = double >
		class Matrix {
		public:
			
			template < unsigned int IM, unsigned int IN, typename IT >
			class MatrixInitializer {
			public:
				MatrixInitializer(Matrix<IM, IN, IT>& mat, unsigned int index = 0) : mat(mat), index(index) { }
				
				MatrixInitializer<IM, IN, IT> operator,(IT right) {
					mat(index / N, index % N) = right;
					return MatrixInitializer<IM, IN, IT>(mat, index + 1);
				}
				
				Matrix<IM, IN, IT>& mat;
				unsigned int index;
			}; // END class MatrixInitializer
		
			Matrix() { }
			
			Matrix(const T v) {
				(*this) = v;
			}
			
			/********** Initializing the matrix **********/
			
			void zero() {
				for(uint8_t i = 0; i < M * N; i++) {
					value[i] = 0;
				}
			}
			
			void eye() {
				zero();
				uint8_t j = (M < N) ? M : N;
				for(uint8_t i = 0; i < j; i++) {
					(*this)(i, i) = 1;
				}
			}
			
			void rotx(double angle) {
				Matrix<M, N, T>& m = *this;
				if(M == 3 && N == 3) {
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
				Matrix<M, N, T>& m = *this;
				if(M == 3 && N == 3) {
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
				Matrix<M, N, T>& m = *this;
				if(M == 3 && N == 3) {
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
			
			MatrixInitializer<M, N, T> operator<<(T right) {
				(*this)[0] = right;
				return MatrixInitializer<M, N, T>(*this, 1);
			}
			
			/********** Element access **********/
			
			const T get(uint8_t m, uint8_t n) const {
				return (*this)(m, n);
			}
			
			Matrix<M, 1, T> getCol(uint8_t n) const {
				Matrix<M, 1, T> col;
				for(uint8_t m = 0; m < M; m++) {
					col(m, 0) = (*this)(m, n);
				}
				return col;
			}
			
			Matrix<1, N, T> getRow(uint8_t m) const {
				Matrix<1, N, T> row;
				for(uint8_t n = 0; n < N; n++) {
					row(0, n) = (*this)(m, n);
				}
				return row;
			}
			
			void set(uint8_t m, uint8_t n, T value) {
				(*this)(m, n) = value;
			}
			
			void setCol(uint8_t n, const Matrix<M, 1, T>& col) {
				for(uint8_t m = 0; m < M; m++) {
					(*this)(m, n) = col(m, 0);
				}
			}
			
			void setRow(uint8_t m, const Matrix<1, N, T>& row) {
				for(uint8_t n = 0; n < N; n++) {
					(*this)(m, n) = row(0, n);
				}
			}
			
			T& operator()(uint8_t m, uint8_t n) {
				if(m >= 0 && m < M && n >= 0 && n < N) {
					return value[M * n + m];
				}
				else {
					throw MatrixIndexOutOfBoundException(m, M, n, N);
				}
			}
			
			const T operator()(uint8_t m, uint8_t n) const {
				if(m >= 0 && m < M && n >= 0 && n < N) {
					return value[M * n + m];
				}
				else {
					throw MatrixIndexOutOfBoundException(m, M, n, N);
				}
			}
			
			T& operator()(unsigned int i) {
				if(i >= 0 && i < M * N) {
					return value[i];
				}
				else {
					throw MatrixIndexOutOfBoundException(i, M * N);
				}
			}
			
			const T operator()(unsigned int i) const {
				if(i >= 0 && i < M * N) {
					return value[i];
				}
				else {
					throw MatrixIndexOutOfBoundException(i, M * N);
				}
			}
			
			T& operator[](unsigned int i) {
				if(i >= 0 && i < M * N) {
					return value[i];
				}
				else {
					throw MatrixIndexOutOfBoundException(i, M * N);
				}
			}
			
			const T operator[](unsigned int i) const {
				if(i >= 0 && i < M * N) {
					return value[i];
				}
				else {
					throw MatrixIndexOutOfBoundException(i, M * N);
				}
			}
			
			/********** Matrix characteristics **********/
			
			constexpr bool isSquare() const {
				return M == N;
			}
			
			bool isOrthogonal() const {
				Matrix<N, M, T> result, transposed, eye;
				eye.eye();
				transposed = this->transpose();
				result = (*this) * transposed;
				return result == eye;
			}
			
			bool isSymmetric() const {
				return (*this) == this->transpose();
			}
			
			bool isDiagonal() const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if(m != n){
							if((*this)(m, n) != 0){
								return false;
							}
						}
					}
				}
				return true;
			}
			
			bool isLowerTriangular() const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if(m < n){
							if((*this)(m, n) != 0){
								return false;
							}
						}
					}
				}
				return true;
			}
			
			bool isUpperTriangular() const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if(m > n){
							if((*this)(m, n) != 0){
								return false;
							}
						}
					}
				}
				return true;
			}
			
			bool isInvertible() const {
				// non square matrices can't be inverted, if the determinat of a matrix is zero it is not invertible
				return (M == N) && (this->det() != 0);
			}
			
			constexpr uint8_t getNofRows() const {
				return M;
			}
			
			constexpr uint8_t getNofColums() const {
				return N;
			}
			
			unsigned int rank() const {
				unsigned int numberOfNonZeroRows = 0;
				Matrix<M, N, T> temp = (*this);
				temp.gaussRowElimination();
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if(temp(m, n) != 0){
							numberOfNonZeroRows++;
							break;
						}
					}
				}
				return numberOfNonZeroRows;
			}
			
			T det() const {
				if(M == N) { // Determinat can only be calculated of a square matrix
					if(M == 2) { // 2x2 matrix
						return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1,0);
					}
					else if(M == 3) { // 3x3 matrix
						T det = 0;
						det = (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 2) +
						      (*this)(0, 1) * (*this)(1, 2) * (*this)(2, 0) +
						      (*this)(0, 2) * (*this)(1, 0) * (*this)(2, 1) -
						      (*this)(0, 2) * (*this)(1, 1) * (*this)(2, 0) -
						      (*this)(0, 0) * (*this)(1, 2) * (*this)(2, 1) -
						      (*this)(0, 1) * (*this)(1, 0) * (*this)(2, 2);
						return det;
					}
					else { // 4x4 and bigger square matrices
						   // Use recurcive laplace formula to calculate the determinat.
						   // For big matrices this method needs a lot of time, this 
						   // could be improved with another algorithm.
						T det = 0;
						uint8_t  ignoredRow = 0;
						for(uint8_t m = 0; m < M; m++) {
							Matrix<M - 1, N - 1, T> subMatrix;
							uint8_t x = 0, y = 0;
							uint8_t a = 0, b = 1;
							while(a < M) {
								while(b < N) {
									if(a != ignoredRow) {
										subMatrix(y, x) = (*this)(a, b);
										x++;
									}
									b++;
								}
								b = 1;
								x = 0;
								if(a != ignoredRow) {
									y++; 
								}
								a++;
							}
							ignoredRow++;
							T detSubMatrix = subMatrix.det();
							if(m % 2 == 0) { // even
								det = det + (*this)(m, 0) * detSubMatrix;
							}
							else { // odd
								det = det - (*this)(m, 0) * detSubMatrix; 
							}
						}
						return det;
					}
				}
				else {
					throw EEROSException("Calculating determinant failed: Matrix must be square"); 
				}
				return 0;
			}
			
			T trace() const {
				T result = 0;
				uint8_t j = (M < N) ? M : N;
				for(uint8_t i = 0; i < j; i++) {
					result += (*this)(i, i);
				}
				return result;
			}
			
			Matrix<N, M, T> transpose() const {
				Matrix<N, M, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						result(n, m) = (*this)(m, n);
					}
				}
				return result;
			}
			
			/********** Base operations **********/
			
			bool operator==(const Matrix<M, N, T>& right) const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if((*this)(m, n) != right(m, n))
							return false;
					}
				}
				return true;
			}

			bool operator!=(const Matrix<M, N, T>& right) const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if((*this)(m, n) != right(m, n)) {
							return true;
						}
					}
				}
				return false;
			}
			
			bool operator<(const Matrix<M, N, T>& right) const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if((*this)(m, n) >= right(m, n)) {
							return false;
						}
					}
				}
				return true;
			}
			
			bool operator<=(const Matrix<M, N, T>& right) const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if((*this)(m, n) > right(m, n)) {
							return false;
						}
					}
				}
				return true;
			}
			
			bool operator>(const Matrix<M, N, T>& right) const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if((*this)(m, n) <= right(m, n)) {
							return false;
						}
					}
				}
				return true;
			}
			
			bool operator>=(const Matrix<M, N, T>& right) const {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						if((*this)(m, n) < right(m, n)) {
							return false;
						}
					}
				}
				return true;
			}
			
			Matrix<M, N, T>& operator=(T right) {
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						(*this)(m, n) = right;
					}
				}
				return *this;
			}
			
			template < uint8_t K >
			Matrix<M, K, T> operator*(const Matrix<N, K, T> right) const {
				Matrix<M, K, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t k = 0; k < K; k++) {
						result(m, k) = 0;
						for(uint8_t n = 0; n < N; n++) {
							result(m, k) += (*this)(m, n) * right(n, k);
						}
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator*(T right) const {
				Matrix<M, N, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						result(m,n) = (*this)(m,n) * right;
					}
				}
				return result;
			}
			
			Matrix<M, N, T> multiplyElementWise(const Matrix<M, N, T> right) const {
			    Matrix<M, N, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) * right(m, n);
					}
			    }
			    return result;
			}
			
			Matrix<M, N, T> operator+(const Matrix<M, N, T> right) const {
				Matrix<M, N, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) + right(m, n);
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator+(const T right) const {
				Matrix<M, N, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) + right;
					}
				}
				return result;
			}
			
			Matrix<M, N, T>& operator+=(const Matrix<M, N, T> right) {
				(*this) = (*this) + right;
				return (*this);
			}
			
			Matrix<M, N, T> operator-(const Matrix<M, N, T> right) const {
				Matrix<M, N, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) - right(m, n);
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator-(const T right) const {
				Matrix<M, N, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) - right;
					}
				}
				return result;
			}
			
			Matrix<M, N, T>& operator-=(const Matrix<M, N, T> right) {
				(*this) = (*this) - right;
				return (*this);
			}
			
			Matrix<M, N, T>& operator-() {
				(*this) = 0 - (*this);
				return (*this);
			}
			
			Matrix<M, N, T> operator/(T right) const {
				Matrix<M, N, T> result;
				for(uint8_t m = 0; m < M; m++) {
					for(uint8_t n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) / right;
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator!() const {
				T determinant = this->det();
				if(N != M) {
					throw EEROSException("Invert failed: matrix not square");
				}
				else if(determinant == 0) {
					throw EEROSException("Invert failed: determinat of matrix is 0");
				}
				else if(this->isOrthogonal() == true) {
					return transpose();
				}
				else if(M == 2) { // 2x2 matrix
					Matrix<M, N, T> result;
					result(0, 0) =  (*this)(1, 1);
					result(1, 0) = -(*this)(1, 0);
					result(0, 1) = -(*this)(0, 1);
					result(1, 1) =  (*this)(0, 0);
					return result / determinant;
				}
				else if(M == 3) { // 3x3 matrix
					Matrix<M, N, T> result;
					result(0, 0) = (*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1);
					result(1, 0) = (*this)(1, 2) * (*this)(2, 0) - (*this)(1, 0) * (*this)(2, 2);
					result(2, 0) = (*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0);
					result(0, 1) = (*this)(0, 2) * (*this)(2, 1) - (*this)(0, 1) * (*this)(2, 2);
					result(1, 1) = (*this)(0, 0) * (*this)(2, 2) - (*this)(0, 2) * (*this)(2, 0);
					result(2, 1) = (*this)(0, 1) * (*this)(2, 0) - (*this)(0, 0) * (*this)(2, 1);
					result(0, 2) = (*this)(0, 1) * (*this)(1, 2) - (*this)(0, 2) * (*this)(1, 1);
					result(1, 2) = (*this)(0, 2) * (*this)(1, 0) - (*this)(0, 0) * (*this)(1, 2);
					result(2, 2) = (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
					return result / determinant;
				}
				else {
					// This algorithm needs a lot of time maybe there is a better one?
					Matrix<M, N, T> result;
					Matrix<M - 1, N - 1, T> smallerPart;
					uint8_t ignoredRow = 0, ignoredColum = 0;
					for(uint8_t m = 0; m < M; m++) {
						for(uint8_t n = 0; n < N; n++) {
							uint8_t a = 0, b = 0;
							// 1. Create "matrix of minors"
							for(uint8_t u = 0; u < M; u++) {
								for(uint8_t w = 0; w < N; w++) {
									if(u != m && w != n){
										smallerPart(a, b) = (*this)(u, w);
										b++;
										b = b % (N - 1);
									}
								}
								if(u != m) {
									a++;
									a = a&(M - 1);
								}
							}
							// 2. Swapp signs 
							if(m % 2 == 0) {
								if(n % 2 != 0) {
									result(m,n) = -smallerPart.det();
								}
								else {
									result(m, n) = smallerPart.det();
								}
							}
							else {
								if(n % 2 != 0){
									result(m, n) = smallerPart.det();
								}
								else {
									result(m, n) = -smallerPart.det();
								}
							}
							// 3. Divide throu det of the original matrix
							result(m, n) = result(m, n) / determinant;
						}
					}
					// 4. transpose
					return result.transpose();
				}
			}
			
			/********** Static functions **********/
			
			static Matrix<M, N, T> createRotX(double angle) {
				Matrix<M, N, T> m;
				m.rotx(angle);
				return m;
			}
			
			static Matrix<M, N, T> createRotY(double angle) {
				Matrix<M, N, T> m;
				m.roty(angle);
				return m;
			}
			
			static Matrix<M, N, T> createRotZ(double angle) {
				Matrix<M, N, T> m;
				m.rotz(angle);
				return m;
			}
			
			static Matrix<2, 1, T> createVector2(T x, T y) {
				Matrix<2, 1, T> v;
				v(0) = x;
				v(1) = y;
				return v;
			}
			
			static Matrix<3, 1, T> createVector3(T x, T y, T z) {
				Matrix<3, 1, T> v;
				v(0) = x;
				v(1) = y;
				v(2) = z;
				return v;
			}
			
			static Matrix<M, N, T> createDiag(T v) {
				Matrix<M, N, T> d;
				d.eye();
				return d * v;
			}
			
			static Matrix<3, 3, T> createSkewSymmetric(Matrix<3, 1, T> a) {
				Matrix<3, 3, T> result;
				result(0, 0) =  0;
				result(0, 1) = -a(2);
				result(0, 2) =  a(1);
				result(1, 0) =  a(2);
				result(1, 1) =  0;
				result(1, 2) = -a(0);
				result(2, 0) = -a(1);
				result(2, 1) =  a(0);
				result(2, 2) =  0;
				return result;
			}
			
			static Matrix<3, 1, T> crossProduct(Matrix<3, 1, T> a, Matrix<3, 1, T> b) {
			    Matrix<3, 1, T> result;
			    result(0, 0) = a(1, 0) * b(2, 0) - a(2, 0) * b(1, 0);
			    result(1, 0) = a(2, 0) * b(0, 0) - a(0, 0) * b(2, 0);
			    result(2, 0) = a(0, 0) * b(1, 0) - a(1, 0) * b(0, 0); 
			    return result;
			}
			
			/********** Helper functions **********/
			
			void gaussRowElimination() {
				uint8_t completedColum = 0, completedRow = 0;
				uint8_t checkingRow = 0, rootRow = 0;
				T rowFactor = 0;
				
				sortForGaussAlgorithm();
				while(completedColum < N) {
					rootRow = completedRow;
					checkingRow = rootRow + 1;
					while(checkingRow < M && (*this)(rootRow, completedColum) != 0) {
						if((*this)(checkingRow, completedColum) != 0) {
							rowFactor = (*this)(checkingRow, completedColum) / (*this)(rootRow, completedColum); 
							for(uint8_t n = completedColum; n < N; n++) {
								(*this)(checkingRow, n) = (*this)(checkingRow, n) - rowFactor * (*this)(rootRow, n);
							}
						}
						checkingRow++;
					}
					completedRow++;
					completedColum++;
				}
			}
			
			void sortForGaussAlgorithm() {
				uint8_t completedColum = 0, completedRow = 0;
				uint8_t swapRow = completedRow + 1;
				
				while(completedColum < N) {
					while(completedRow < M) {
						if((*this)(completedRow, completedColum) == 0 && swapRow < M && completedRow < M - 1) {
							swapRows(completedRow, swapRow);
							swapRow++;
						}
						else {
							completedRow++; 
						}
				  }
				  completedColum++;
				  swapRow = completedRow + 1;
				}
			}
			
			void swapRows(uint8_t rowA, uint8_t rowB) {
				for(uint8_t n = 0; n < N; n++) {
					T t = (*this)(rowA, n);
					(*this)(rowA, n) = (*this)(rowB, n);
					(*this)(rowB, n) =  t;
				}
			}
			
		protected:
			
			T value[M * N];
			
		}; // END class Matrix
		
		/********** Operators **********/
		
		template < uint8_t M, uint8_t N = 1, typename T = double >
		Matrix<M, N, T> operator+(T left, Matrix<M, N, T> right) {
			Matrix<M, N, T> result;
			for(uint8_t m = 0; m < M; m++) {
				for(uint8_t n = 0; n < N; n++) {
					result(m, n) = left + right(m, n);
				}
			}
			return result;
		}
		
		template < uint8_t M, uint8_t N = 1, typename T = double >
		Matrix<M, N, T> operator-(T left, Matrix<M, N, T> right) {
			Matrix<M, N, T> result;
			for(uint8_t m = 0; m < M; m++) {
				for(uint8_t n = 0; n < N; n++) {
					result(m, n) = left - right(m, n);
				}
			}
			return result;
		}
		
		template < uint8_t M, uint8_t N = 1, typename T = double >
		Matrix<M, N, T> operator*(T left, Matrix<M, N, T> right) {
			Matrix<M, N, T> result;
			for(uint8_t m = 0; m < M; m++) {
				for(uint8_t n = 0; n < N; n++) {
					result(m, n) = left * right(m, n);
				}
			}
			return result;
		}
		
		template < uint8_t M, uint8_t N = 1, typename T = double >
		Matrix<M, N, T> operator/(T left, Matrix<M, N, T> right) {
			Matrix<M, N, T> result;
			for(uint8_t m = 0; m < M; m++) {
				for(uint8_t n = 0; n < N; n++) {
					result(m, n) = left / right(m, n);
				}
			}
			return result;
		}
		
		/********** Print functions **********/
		
		template < uint8_t M, uint8_t N = 1, typename T = double >
		std::ostream& operator<<(std::ostream& os, const Matrix<M, N, T>& right) {
			if(N > 1) os << "[ ";
			for(uint8_t n = 0; n < N; n++) {
				os << '[';
				for(uint8_t m = 0; m < M; m++) {
					os << right(m, n);
					if(m < M - 1) os << ' ';
				}
				os << "]' ";
			}
			if(N > 1) os << "]";
			return os;
		}
		
		/********** Type definitions **********/
		
		typedef Matrix<2,1> Vector2;
		typedef Matrix<3,1> Vector3;
		typedef Matrix<4,1> Vector4;
		
	} // END namespace math
} // END namespache eeros

#endif /* ORG_EEROS_MATH_MATRIX_HPP_ */
