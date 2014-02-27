#ifndef ORG_EEROS_CONTROL_MATRIXMULTIPLICATION_HPP_
#define ORG_EEROS_CONTROL_MATRIXMULTIPLICATION_HPP_

#include <vector>

#include <eeros/core/Matrix.hpp>
#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {

		class MatrixMultiplication: public Block1i1o {
		public:
			//template < int N, int M>
			MatrixMultiplication();
			virtual ~MatrixMultiplication();

			virtual void run();

		protected:
			eeros::math::Matrix<2,4> m;
			//template < int N, int M>
			//eeros::math::Matrix<N,M> m;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_MATRIXMULTIPLICATION_HPP_ */
