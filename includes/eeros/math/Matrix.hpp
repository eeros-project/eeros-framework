#ifndef ORG_EEROS_MATH_MATRIX_HPP_
#define ORG_EEROS_MATH_MATRIX_HPP_

#include <eeros/core/Fault.hpp>
#include "MatrixIndexOutOfBoundException.hpp"

#include <utility>
#include <sstream>
#include <cstdlib>
#include <cmath>

namespace eeros {
	namespace math {
		template < unsigned int M, unsigned int N = 1, typename T = double >
		class Matrix {
		public:
			
			static_assert((M > 1 && N >= 1) || (M >=1 && N > 1), "Matrix dimension must be greater or equal than 1x1!");
			
			using value_type = T;
			
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
			
			/********** Constructors **********/
			
			Matrix() { }
			
			Matrix(const T v) {
				(*this) = v;
			}
			
			template<typename... S>
			Matrix(const S... v) : value{std::forward<const T>(v)...} {
				static_assert(sizeof...(S) == M * N, "Invalid number of constructor arguments!");
			}
			
			/********** Initializing the matrix **********/
			
			void zero() {
				for(unsigned int i = 0; i < M * N; i++) {
					value[i] = 0;
				}
			}
			
			void eye() {
				zero();
				unsigned int j = (M < N) ? M : N;
				for(unsigned int i = 0; i < j; i++) {
					(*this)(i, i) = 1;
				}
			}
			
			void fill(T v) {
				(*this) = v;
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
					throw Fault("rotx(double) is only implemented for 3x3 matrices");
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
					throw Fault("roty(double) is only implemented for 3x3 matrices");
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
					throw Fault("rotz(double) is only implemented for 3x3 matrices");
				}
			}
			
			MatrixInitializer<M, N, T> operator<<(T right) {
				(*this)[0] = right;
				return MatrixInitializer<M, N, T>(*this, 1);
			}
			
			/********** Element access **********/
			
			const T get(unsigned int m, unsigned int n) const {
				return (*this)(m, n);
			}
			
			Matrix<M, 1, T> getCol(unsigned int n) const {
				Matrix<M, 1, T> col;
				for(unsigned int m = 0; m < M; m++) {
					col(m, 0) = (*this)(m, n);
				}
				return col;
			}
			
			Matrix<1, N, T> getRow(unsigned int m) const {
				Matrix<1, N, T> row;
				for(unsigned int n = 0; n < N; n++) {
					row(0, n) = (*this)(m, n);
				}
				return row;
			}
			
			template<unsigned int U, unsigned int V>
			Matrix<U, V, T> getSubMatrix(unsigned int m, unsigned int n) const {
				static_assert(U <= M && V <= N, "Dimension of the sub matrix must be lower or equal than of the origin!");
				if(m + U <= M && n + V <= N) {
					Matrix<U, V, T> sub;
					for(unsigned int u = 0; u < U; u++) {
						for(unsigned int v = 0; v < V; v++) {
							sub(u, v) = (*this)(m + u, n + v);
						}
					}
					return sub;
				}
				else {
					throw MatrixIndexOutOfBoundException(m + U - 1, M, n + V - 1, N);
				}
			}
			
			void set(unsigned int m, unsigned int n, T value) {
				(*this)(m, n) = value;
			}
			
			void setCol(unsigned int n, const Matrix<M, 1, T>& col) {
				for(unsigned int m = 0; m < M; m++) {
					(*this)(m, n) = col(m, 0);
				}
			}
			
			void setRow(unsigned int m, const Matrix<1, N, T>& row) {
				for(unsigned int n = 0; n < N; n++) {
					(*this)(m, n) = row(0, n);
				}
			}
			
			T& operator()(unsigned int m, unsigned int n) {
				if(m >= 0 && m < M && n >= 0 && n < N) {
					return value[M * n + m];
				}
				else {
					throw MatrixIndexOutOfBoundException(m, M, n, N);
				}
			}
			
			const T operator()(unsigned int m, unsigned int n) const {
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
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
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
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
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
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
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
			
			constexpr unsigned int getNofRows() const {
				return M;
			}
			
			constexpr unsigned int getNofColums() const {
				return N;
			}
			
			constexpr unsigned int size() const {
				return M * N;
			}
			
			unsigned int rank() const {
				unsigned int numberOfNonZeroRows = 0;
				Matrix<M, N, T> temp = (*this);
				temp.gaussRowElimination();
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
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
						unsigned int  ignoredRow = 0;
						for(unsigned int m = 0; m < M; m++) {
							Matrix<M - 1, N - 1, T> subMatrix;
							unsigned int x = 0, y = 0;
							unsigned int a = 0, b = 1;
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
					throw Fault("Calculating determinant failed: Matrix must be square"); 
				}
				return 0;
			}
			
			T trace() const {
				T result = 0;
				unsigned int j = (M < N) ? M : N;
				for(unsigned int i = 0; i < j; i++) {
					result += (*this)(i, i);
				}
				return result;
			}
			
			Matrix<N, M, T> transpose() const {
				Matrix<N, M, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						result(n, m) = (*this)(m, n);
					}
				}
				return result;
			}
			
			/********** Base operations **********/
			
			bool operator==(const Matrix<M, N, T>& right) const {
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						if((*this)(m, n) != right(m, n))
							return false;
					}
				}
				return true;
			}

			bool operator!=(const Matrix<M, N, T>& right) const {
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						if((*this)(m, n) != right(m, n)) {
							return true;
						}
					}
				}
				return false;
			}
			
			bool operator<(const Matrix<M, N, T>& right) const {
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						if((*this)(m, n) >= right(m, n)) {
							return false;
						}
					}
				}
				return true;
			}
			
			bool operator<=(const Matrix<M, N, T>& right) const {
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						if((*this)(m, n) > right(m, n)) {
							return false;
						}
					}
				}
				return true;
			}
			
			bool operator>(const Matrix<M, N, T>& right) const {
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						if((*this)(m, n) <= right(m, n)) {
							return false;
						}
					}
				}
				return true;
			}
			
			bool operator>=(const Matrix<M, N, T>& right) const {
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						if((*this)(m, n) < right(m, n)) {
							return false;
						}
					}
				}
				return true;
			}
			
			Matrix<M, N, T>& operator=(T right) {
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						(*this)(m, n) = right;
					}
				}
				return *this;
			}
			
			template < unsigned int K >
			Matrix<M, K, T> operator*(const Matrix<N, K, T> right) const {
				Matrix<M, K, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int k = 0; k < K; k++) {
						result(m, k) = 0;
						for(unsigned int n = 0; n < N; n++) {
							result(m, k) += (*this)(m, n) * right(n, k);
						}
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator*(T right) const {
				Matrix<M, N, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						result(m,n) = (*this)(m,n) * right;
					}
				}
				return result;
			}
			
			Matrix<M, N, T> multiplyElementWise(const Matrix<M, N, T> right) const {
			    Matrix<M, N, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) * right(m, n);
					}
			    }
			    return result;
			}
			
			Matrix<M, N, T> operator+(const Matrix<M, N, T> right) const {
				Matrix<M, N, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) + right(m, n);
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator+(const T right) const {
				Matrix<M, N, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
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
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) - right(m, n);
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator-(const T right) const {
				Matrix<M, N, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) - right;
					}
				}
				return result;
			}
			
			Matrix<M, N, T>& operator-=(const Matrix<M, N, T> right) {
				(*this) = (*this) - right;
				return (*this);
			}
			
			Matrix<M, N, T> operator-() {
				Matrix<M, N, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						result(m, n) = -(*this)(m, n);
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator/(T right) const {
				Matrix<M, N, T> result;
				for(unsigned int m = 0; m < M; m++) {
					for(unsigned int n = 0; n < N; n++) {
						result(m, n) = (*this)(m, n) / right;
					}
				}
				return result;
			}
			
			Matrix<M, N, T> operator!() const {
				T determinant = this->det();
				if(N != M) {
					throw Fault("Invert failed: matrix not square");
				}
				else if(determinant == 0) {
					throw Fault("Invert failed: determinat of matrix is 0");
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
					unsigned int ignoredRow = 0, ignoredColum = 0;
					for(unsigned int m = 0; m < M; m++) {
						for(unsigned int n = 0; n < N; n++) {
							unsigned int a = 0, b = 0;
							// 1. Create "matrix of minors"
							for(unsigned int u = 0; u < M; u++) {
								for(unsigned int w = 0; w < N; w++) {
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
				unsigned int completedColum = 0, completedRow = 0;
				unsigned int checkingRow = 0, rootRow = 0;
				T rowFactor = 0;
				
				sortForGaussAlgorithm();
				while(completedColum < N) {
					rootRow = completedRow;
					checkingRow = rootRow + 1;
					while(checkingRow < M && (*this)(rootRow, completedColum) != 0) {
						if((*this)(checkingRow, completedColum) != 0) {
							rowFactor = (*this)(checkingRow, completedColum) / (*this)(rootRow, completedColum); 
							for(unsigned int n = completedColum; n < N; n++) {
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
				unsigned int completedColum = 0, completedRow = 0;
				unsigned int swapRow = completedRow + 1;
				
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
			
			void swapRows(unsigned int rowA, unsigned int rowB) {
				for(unsigned int n = 0; n < N; n++) {
					T t = (*this)(rowA, n);
					(*this)(rowA, n) = (*this)(rowB, n);
					(*this)(rowB, n) =  t;
				}
			}
			
		protected:
			T value[M * N];
			
		}; // END class Matrix
		
		/********** Operators **********/
		
		template < unsigned int M, unsigned int N = 1, typename T = double >
		Matrix<M, N, T> operator+(T left, Matrix<M, N, T> right) {
			Matrix<M, N, T> result;
			for(unsigned int m = 0; m < M; m++) {
				for(unsigned int n = 0; n < N; n++) {
					result(m, n) = left + right(m, n);
				}
			}
			return result;
		}
		
		template < unsigned int M, unsigned int N = 1, typename T = double >
		Matrix<M, N, T> operator-(T left, Matrix<M, N, T> right) {
			Matrix<M, N, T> result;
			for(unsigned int m = 0; m < M; m++) {
				for(unsigned int n = 0; n < N; n++) {
					result(m, n) = left - right(m, n);
				}
			}
			return result;
		}
		
		template < unsigned int M, unsigned int N = 1, typename T = double >
		Matrix<M, N, T> operator-(const Matrix<M, N, T> &right) {
			Matrix<M, N, T> result;
			for(unsigned int m = 0; m < M; m++) {
				for(unsigned int n = 0; n < N; n++) {
					result(m, n) = -right(m, n);
				}
			}
			return result;
		}
		
		template < unsigned int M, unsigned int N = 1, typename T = double >
		Matrix<M, N, T> operator*(T left, Matrix<M, N, T> right) {
			Matrix<M, N, T> result;
			for(unsigned int m = 0; m < M; m++) {
				for(unsigned int n = 0; n < N; n++) {
					result(m, n) = left * right(m, n);
				}
			}
			return result;
		}
		
		template < unsigned int M, unsigned int N = 1, typename T = double >
		Matrix<M, N, T> operator/(T left, Matrix<M, N, T> right) {
			Matrix<M, N, T> result;
			for(unsigned int m = 0; m < M; m++) {
				for(unsigned int n = 0; n < N; n++) {
					result(m, n) = left / right(m, n);
				}
			}
			return result;
		}
		
		/********** Print functions **********/
		
		template < unsigned int M, unsigned int N = 1, typename T = double >
		std::ostream& operator<<(std::ostream& os, const Matrix<M, N, T>& right) {
			if(N > 1) os << "[ ";
			for(unsigned int n = 0; n < N; n++) {
				os << '[';
				for(unsigned int m = 0; m < M; m++) {
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
		
		template < unsigned int M, typename T = double >
		using Vector = Matrix<M, 1, T>;
		
		/********** Specialization for a 1x1 matrix **********/
		
		template < typename T >
		class Matrix<1, 1, T> {
			
		public:
			
			using value_type = T;
			
			Matrix() { }
			
			Matrix(const T v) { value = v; }
			
			void zero() { value = 0; }
			
			void eye() { value = 1; }
			
			void fill(T v) { value = v; }
			
			const T get(uint8_t m, uint8_t n) const { return (*this)(m, n); }
			
			Matrix<1, 1, T> getCol(uint8_t n) const { return (*this); }
			
			Matrix<1, 1, T> getRow(uint8_t m) const { return (*this); }
			
			void set(uint8_t m, uint8_t n, T value) { (*this)(m, n) = value; }
			
			T& operator()(uint8_t m, uint8_t n) { if(m == 0 && n == 0) return value; else throw MatrixIndexOutOfBoundException(m, 1, n, 1); }
			
			const T operator()(uint8_t m, uint8_t n) const { if(m == 0 && n == 0) return value; else throw MatrixIndexOutOfBoundException(m, 1, n, 1); }
			
			T& operator()(unsigned int i) { if(i == 0) return value; else throw MatrixIndexOutOfBoundException(i, 1); }
			
			const T operator()(unsigned int i) const { if(i == 0) return value; else throw MatrixIndexOutOfBoundException(i, 1); }
			
			T& operator[](unsigned int i) { if(i == 0) return value; else throw MatrixIndexOutOfBoundException(i, 1); }
			
			const T operator[](unsigned int i) const { if(i == 0) return value; else throw MatrixIndexOutOfBoundException(i, 1); }
			
			constexpr bool isSquare() const { return true; }
			
			bool isOrthogonal() const { return value == 1; }
			
			constexpr bool isSymmetric() const { return true; }
			
			constexpr bool isDiagonal() const { return true; }
			
			constexpr bool isLowerTriangular() const { return true; }
			
			constexpr bool isUpperTriangular() const { return true; }
			
			bool isInvertible() const { return value != 0; }
			
			constexpr unsigned int getNofRows() const { return 1; }
			
			constexpr unsigned int getNofColums() const { return 1; }
			
			constexpr unsigned int size() const { return 1; }
			
			unsigned int rank() const { return (value == 0) ? 1 : 0; }
			
			T det() const { return value; }
			
			T trace() const { return value; }
			
			Matrix<1, 1, T> operator!() const { Matrix<1, 1, T> inv(1/value); return inv; }
			
			operator T() const { return value; }
			
			Matrix<1, 1, T>& operator+=(const Matrix<1, 1, T> right) {
				(*this) = (*this) + right;
				return (*this);
			}
			
			Matrix<1, 1, T>& operator-=(const Matrix<1, 1, T> right) {
				(*this) = (*this) - right;
				return (*this);
			}
			
		protected:
			T value;
		};
		
	} // END namespace math
} // END namespache eeros

#endif /* ORG_EEROS_MATH_MATRIX_HPP_ */
