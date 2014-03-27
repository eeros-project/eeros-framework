#ifndef ORG_EEROS_CORE_MATRIXINDEXOUTOFBOUNDEXCEPTION_HPP
#define ORG_EEROS_CORE_MATRIXINDEXOUTOFBOUNDEXCEPTION_HPP

#include <eeros/core/EEROSException.hpp>

namespace eeros {
	namespace math {
		class MatrixIndexOutOfBoundException : public eeros::EEROSException {

		public:
			MatrixIndexOutOfBoundException(unsigned int m, unsigned int M, unsigned int n, unsigned int N);
			virtual ~MatrixIndexOutOfBoundException() throw();
		};
	};
};

#endif // ORG_EEROS_CORE_MATRIXINDEXOUTOFBOUNDEXCEPTION_HPP
